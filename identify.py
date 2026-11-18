"""Decode an IMU-based PRBS or multi-speed friction dump (hex Size 12080).

Optional --fit estimates local J,b with Coulomb, stiffness and offset nuisance
terms. Results are candidates: a second capture with a new seed is validation.
"""
import argparse
import csv
import json
from pathlib import Path
import struct

SAMPLE = struct.Struct('<6fII')
CAPACITY = 1536
DUMP_BYTES = 0xC008
COLUMNS = ['time_s','position_rad','omega_imu_rad_s','omega_motor_rad_s',
           'torque_cmd_mean_nm','torque_feedback_mean_nm','stage','flags']

def decode_legacy32(raw):
    import math
    if len(raw) != DUMP_BYTES:
        raise ValueError('Export &g_sysid.buf_[0], hexadecimal Size C008 (49160 bytes).')
    index,count=struct.unpack_from('<II',raw,0xC000)
    if index != count or not 2 <= count <= CAPACITY:
        raise ValueError('Invalid frozen linear capture counters.')
    rows=[]
    for i in range(count):
        t,u,q,w,ref,fb,stage,flags=SAMPLE.unpack_from(raw,i*32)
        if stage not in (400,401,403) or flags & ~1 or not all(math.isfinite(v) for v in [t,u,q,w,fb]):
            raise ValueError('Not a new PRBS capture; old calibration/long-speed dumps require the old tools.')
        if rows and t <= rows[-1][0]: raise ValueError('Non-increasing time.')
        rows.append((t,q,w,ref,u,fb,0.,0.,0.,0.,{400:0,401:1,403:4}[stage],flags))
    exciting=[r[0] for r in rows if r[10]==1]
    return rows,dict(center=0.,settle_s=1.,
                     excite_s=max(exciting)-1. if exciting else 0.,status=None,
                     status_text='completion not stored; check Ozone status_ = 2',count=count,
                     velocity_source='signed IMU gyro axis; motor speed recorded for comparison',format_version='imu32-prbs')


def decode(raw):
    import math
    if len(raw)==0xC008:
        return decode_legacy32(raw)
    if len(raw)!=0x12080:
        raise ValueError('Restored capture: export &g_sysid.capture_, hex Size 12080 (73856 bytes).')
    magic,version,size,capacity,count,status,seed,reserved=struct.unpack_from('<8I',raw)
    if magic!=0x59534944 or version not in (2,3) or size!=48 or capacity!=1536:
        raise ValueError('Wrong restored capture header/address/firmware.')
    if status in (0,1) or not 2<=count<=1536:
        raise ValueError('Capture is still running or contains too few samples.')
    names=['center','min_position','max_position','amplitude','kp','kd','torque_limit','max_travel','settle_s','excite_s','return_s','log_period_s','hold_fast_s','hold_mid_s','hold_slow_s','torque_constant','command_quantum']
    meta=dict(zip(names,struct.unpack_from('<17f',raw,32)))
    meta.update(status=status,status_text='done' if status==2 else 'stopped status '+str(status),count=count,seed=seed,format_version='interval-v3' if version==3 else 'restored-v2')
    friction = bool(reserved & 1)
    if (reserved & 0xfe) or (not friction and reserved) or (friction and version != 3):
        raise ValueError('Unknown capture mode/flags.')
    meta['test_mode'] = 'friction-sweep' if friction else 'prbs'
    if friction:
        stroke, accel, speed_kp, speed_ki = struct.unpack_from('<4f', raw, 112)
        speeds = [meta.pop('amplitude'), meta.pop('hold_fast_s'), meta.pop('hold_mid_s'), meta.pop('hold_slow_s')]
        if not all(math.isfinite(v) and v > 0 for v in [stroke, accel, speed_kp, speed_ki, *speeds]) or not all(a < b for a,b in zip(speeds, speeds[1:])):
            raise ValueError('Corrupt friction sweep settings.')
        if not 2 <= ((reserved >> 8) & 255) <= 5 or (reserved >> 16) > 50000:
            raise ValueError('Corrupt friction sweep repeat count.')
        meta.update(sweep_speeds_rad_s=speeds, sweep_stroke_rad=stroke,
                    sweep_acceleration_rad_s2=accel, sweep_speed_kp=speed_kp,
                    sweep_speed_ki=speed_ki, sweep_repeats=(reserved >> 8) & 255,
                    sweep_speed_filter_hz=(reserved >> 16) / 1000.)
    valid_stages = (0, 5, 6, 7, 10, 11, 12, 13) if friction else (0, 1, 2, 3, 4)
    rows=[struct.unpack_from('<10f2I',raw,128+48*i) for i in range(count)]
    if rows[0][0]!=0:raise ValueError('Initial sample missing.')
    for i,r in enumerate(rows):
        if not all(math.isfinite(v) for v in r[:10]) or r[10] not in valid_stages or r[11]&~7 or (i and r[0]<=rows[i-1][0]):raise ValueError('Corrupt restored sample.')
    return rows,meta


def fit_candidates(rows, metadata, fixed_j=None):
    import numpy as np
    from scipy.integrate import cumulative_trapezoid
    from scipy.optimize import lsq_linear
    from scipy.signal import savgol_filter
    a = np.asarray(rows,dtype=float)
    interval_mode=metadata.get('format_version')=='interval-v3'
    if interval_mode:
        centers=.5*(a[1:,0]+a[:-1,0])
        a=a[1:].copy()
        a[:,0]=centers
        a[:,1]=a[:,9]  # same-interval mean encoder position
    t, q = a[:, 0], a[:, 1] - metadata['center']
    # Apply smoothing only to the velocity endpoints, no noisy differentiation.
    dt = float(np.median(np.diff(t)))
    length = max(5, int(round(.06 / dt)) | 1)
    if len(a) < length + 20:
        raise ValueError('Not enough data for a dynamic fit.')
    # V3 already averaged ALL high-rate IMU samples over the torque interval.
    # Never smooth velocity alone: that gives it a different transfer function.
    w = a[:,2].copy() if interval_mode else savgol_filter(a[:, 2], length, 2)
    sign_basis=np.sign(w)
    if interval_mode:
        # Apply one identical linear filter to EVERY term in the equation.
        sign_basis=savgol_filter(sign_basis,length,2)
        w=savgol_filter(w,length,2)
        q=savgol_filter(q,length,2)
        for col in (4,5):a[:,col]=savgol_filter(a[:,col],length,2)
    terms = np.column_stack([w, sign_basis, q, np.ones(len(t))])
    integrals = cumulative_trapezoid(terms, t, axis=0, initial=0)
    report = {'model': 'tau = J*domega/dt + b*omega + fc*sign(omega) + k*q + offset',
              'state': 'IMU yaw axis; base/big yaw fixed, pitch near calibrated zero and fixed',
              'measurement_alignment': 'common interval centers' if interval_mode else 'legacy endpoints',
              'velocity_source': 'full-rate gyro interval means, matched to torque intervals' if interval_mode else 'signed IMU gyro axis; 60 ms endpoint smoothing',
              'estimator': 'PRBS instrumental-variable integral fit' if interval_mode else 'legacy integral least squares',
              'fits': {}, 'warnings': [
                  'Candidates only; validate with another seed/capture.',
                  'Feedback torque is current-derived using configured Kt, not a torque sensor.',
                  'Sensor and actuator transport delays are not independently calibrated.',
                  'If b changes substantially with window/input, it is not yet identified.']}
    if metadata['status'] != 2:
        report['warnings'].append(metadata['status_text'])
    moving = (a[:, 10] >= 1) & (a[:, 10] <= 3) & (np.abs(w) >= .06)
    instruments=None
    if interval_mode:
        from scipy.signal import lfilter
        excitation=a[:,6]
        z=[np.ones(len(t)),q,np.sign(np.gradient(q,t)),excitation]
        for pole in (2.,5.,10.,20.):
            alpha=np.exp(-pole*dt)
            z.append(lfilter([1-alpha],[1,-alpha],excitation))
        instruments=np.column_stack(z)
        zareas=cumulative_trapezoid(instruments,t,axis=0,initial=0)
    for source, column in [('feedback', 5), ('command', 4)]:
        # Each logged torque is the mean over the interval ENDING at its row.
        area = (cumulative_trapezoid(a[:,column],t,initial=0) if interval_mode
                else np.r_[0., np.cumsum(a[1:, column] * np.diff(t))])
        estimates = []
        for window in [.04, .08, .16]:
            x, y, ends, zrows = [], [], [], []
            for i in range(1, len(t) - 2, 2):
                j = int(np.searchsorted(t, t[i] + window))
                if j >= len(t):
                    continue
                if not np.all((a[i:j+1, 10] >= 1) & (a[i:j+1, 10] <= 3)):
                    continue
                if np.mean(moving[i:j+1]) < .8 or np.any(a[i:j+1, 11].astype(int) & 1):
                    continue
                if np.min(w[i:j+1]) < -.06 and np.max(w[i:j+1]) > .06:
                    continue
                x.append(np.r_[w[j]-w[i], integrals[j]-integrals[i]])
                y.append(area[j]-area[i]); ends.append(t[j])
                if instruments is not None:
                    zrows.append(np.r_[zareas[j]-zareas[i],instruments[j,3:]-instruments[i,3:]])
            if len(x) < 30:
                estimates.append({'window_s': window, 'available': False,
                                  'reason': 'Too few moving windows (sticking/early stop).',
                                  'windows': len(x)})
                continue
            x, y, ends = np.asarray(x), np.asarray(y), np.asarray(ends)
            if fixed_j is not None:
                y = y-fixed_j*x[:,0]
                x = x[:,1:]
            scale = np.linalg.norm(x, axis=0)
            if np.any(scale < 1e-10):
                continue
            lower = np.array([1e-6, 0., 0., -20., -1.])
            upper = np.array([1., 5., .5, 20., 1.])
            if fixed_j is not None:
                lower,upper=lower[1:],upper[1:]
            def solve(mask):
                design,response=x[mask],y[mask]
                if instruments is not None:
                    z=np.asarray(zrows)[mask]
                    zn=np.linalg.norm(z,axis=0)
                    z=z[:,zn>1e-10]/zn[zn>1e-10]
                    # Two-stage least squares: PRBS-based instruments reduce
                    # gyro errors-in-variables and closed-loop correlation.
                    projection=np.linalg.lstsq(z,np.column_stack([design,response]),rcond=1e-6)[0]
                    projected=z@projection
                    design,response=projected[:,:-1],projected[:,-1]
                f = lsq_linear(design/scale, response,
                               bounds=(lower*scale, upper*scale), tol=1e-12)
                return f.x/scale
            p = solve(np.ones(len(x), dtype=bool))
            residual = x @ p - y
            last_stage = ends >= metadata['settle_s'] + 2*metadata['excite_s']/3
            validation = None
            if np.sum(last_stage) >= 20 and np.sum(~last_stage) >= 30:
                train_p = solve(~last_stage)
                validation = float(np.sqrt(np.mean((x[last_stage]@train_p-y[last_stage])**2)))
            params = np.r_[fixed_j,p] if fixed_j is not None else p
            estimates.append(dict(window_s=window, available=True, windows=len(x),
                J_kg_m2=float(params[0]), b_nm_s_rad=float(params[1]),
                coulomb_nuisance_nm=float(params[2]), stiffness_nuisance_nm_rad=float(params[3]),
                offset_nm=float(params[4]), scaled_condition=float(np.linalg.cond(x/scale)),
                integrated_torque_rmse_nm_s=float(np.sqrt(np.mean(residual**2))),
                last_stage_validation_rmse_nm_s=validation,
                parameter_at_bound=bool(np.any(np.isclose(p, lower, atol=1e-5)) or
                                        np.any(np.isclose(p, upper, atol=1e-5)))))
        report['fits'][source] = estimates
    report['quality'] = dict(duration_s=float(t[-1]), sample_interval_median_s=dt,
        position_range_rad=[float(a[:, 1].min()), float(a[:, 1].max())],
        moving_fraction=float(np.mean(moving)),
        speed_range_rad_s=[float(a[:, 2].min()), float(a[:, 2].max())])
    return report


def check_input_delays(rows,metadata,fixed_j=None):
    """Diagnostic candidates selected by held-out-stage residual, never by a target J."""
    import numpy as np
    a=np.asarray(rows,dtype=float)
    result={}
    for source,col in [('command',4),('feedback',5)]:
        candidates=[]
        for advance in (-.02,-.01,0.,.01,.02):
            shifted=a.copy()
            shifted[:,col]=np.interp(a[:,0]+advance,a[:,0],a[:,col])
            report=fit_candidates(shifted.tolist(),metadata,fixed_j)
            for fit in report['fits'][source]:
                error=fit.get('last_stage_validation_rmse_nm_s')
                if fit.get('available') and error is not None and not fit['parameter_at_bound']:
                    candidates.append(dict(input_advance_s=advance,window_s=fit['window_s'],
                        validation_torque_rmse_nm=error/fit['window_s'],
                        J_kg_m2=fit['J_kg_m2'],b_nm_s_rad=fit['b_nm_s_rad']))
        result[source]={'definition':'positive advance uses later torque samples; not proof of a sensor-only delay',
                        'selection':'held-out last-stage residual; independent capture still required',
                        'best_candidate':min(candidates,key=lambda x:x['validation_torque_rmse_nm']) if candidates else None,
                        'candidates':candidates}
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('binary', type=Path)
    p.add_argument('--out', type=Path)
    p.add_argument('--fit', action='store_true')
    p.add_argument('--fixed-j', type=float, help='Optional trusted output-shaft inertia in kg*m^2')
    args = p.parse_args()
    try:
        if args.fixed_j is not None and (not __import__("math").isfinite(args.fixed_j) or args.fixed_j <= 0):
            raise ValueError("fixed-j must be positive and finite")
        rows, metadata = decode(args.binary.read_bytes())
        out = args.out or args.binary.with_suffix('.csv')
        with out.open('w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            friction = metadata.get('test_mode') == 'friction-sweep'
            columns = COLUMNS[:6] + (['velocity_reference_mean_rad_s', 'controller_torque_mean_nm', 'pitch_rad', 'position_mean_rad'] if friction else []) + COLUMNS[6:]
            writer.writerow(columns)
            indices = tuple(range(12)) if friction else (0,1,2,3,4,5,10,11)
            writer.writerows(tuple(r[i] for i in indices) for r in rows)
        out.with_suffix('.metadata.json').write_text(json.dumps(metadata, indent=2), encoding='utf-8')
        if args.fit:
            if metadata.get('test_mode') == 'friction-sweep':
                if args.fixed_j is not None:
                    raise ValueError('command=2 near-steady fit does not use --fixed-j.')
                from friction_fit import fit_friction
                result = fit_friction(rows, metadata)
            else:
                result = fit_candidates(rows, metadata, args.fixed_j)
            if metadata.get('format_version')=='interval-v3' and metadata.get('test_mode') != 'friction-sweep':
                result['input_delay_check']=check_input_delays(rows,metadata,args.fixed_j)
            out.with_suffix('.fit.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
            print(json.dumps(result, indent=2))
        print(f"{metadata['status_text']}: {len(rows)} samples, {rows[-1][0]:.3f} s -> {out}")
    except (OSError, ValueError, ImportError) as error:
        p.exit(1, f'Error: {error}\n')


if __name__ == '__main__':
    main()
