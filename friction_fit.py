"""Near-steady, bidirectional friction fit. No supplied/assumed inertia.

Only constant-reference central traversals with low measured acceleration are
eligible. k*q and offset are nuisance terms for position-dependent loading.
Uncertainty resamples whole reciprocal cycles, not correlated adjacent rows.
"""


def fit_friction(rows, metadata):
    import numpy as np
    from scipy.optimize import lsq_linear

    if metadata.get('test_mode') != 'friction-sweep' or metadata.get('format_version') != 'interval-v3':
        raise ValueError('Friction fit requires an interval-v3 command=2 capture.')
    a = np.asarray(rows, dtype=float)
    times = .5 * (a[1:, 0] + a[:-1, 0])
    a = a[1:].copy()
    t, w, q, ref = times, a[:, 2], a[:, 9] - metadata['center'], a[:, 6]
    stage, flags = a[:, 10].astype(int), a[:, 11].astype(int)
    speeds = np.asarray(metadata['sweep_speeds_rad_s'])
    level = np.clip(stage - 10, 0, 3)
    # Adjacent central differences reject acceleration/braking. This is a
    # selection diagnostic; no differentiated gyro enters the regression.
    alpha, ref_alpha = np.gradient(w, t), np.gradient(ref, t)
    gate = min(.018, .45 * metadata['sweep_stroke_rad'])
    selected = ((stage >= 10) & (stage <= 13) & (np.abs(q) <= gate)
                & (np.abs(w) >= .06) & (w * ref > 0)
                & (np.abs(np.abs(ref) - speeds[level]) <= .03 * speeds[level])
                & (np.abs(w - ref) <= np.maximum(.015, .10 * speeds[level]))
                & (np.abs(alpha) <= .4) & (np.abs(ref_alpha) <= .15)
                & ((flags & 5) == 0))
    selected[0] = selected[-1] = False
    selected[1:-1] &= (stage[:-2] == stage[1:-1]) & (stage[2:] == stage[1:-1])
    # Every moving segment is separated by dwell. Two consecutive legs form
    # a reciprocal cycle, the unit used for repeat-level validation/bootstrap.
    leg = np.full(len(a), -1, dtype=int)
    counters = [0] * 4
    current = -1
    for i, s in enumerate(stage):
        if 10 <= s <= 13:
            if i == 0 or stage[i - 1] != s:
                current = counters[s - 10]
                counters[s - 10] += 1
            leg[i] = (s - 10) * 100 + current
    cycle = np.where(leg >= 0, (leg // 100) * 100 + (leg % 100) // 2, -1)
    # Keep only reciprocal cycles with eligible samples in both directions.
    # A partial last leg must not make a bootstrap sample one-sided.
    for c in np.unique(cycle[selected]):
        keep = selected & (cycle == c)
        if len(np.unique(np.sign(w[keep]))) != 2:
            selected[cycle == c] = False
    report = dict(model='tau = b*omega + fc*sign(omega) + k*q + offset; near-steady only',
                  estimator='balanced robust least squares, reciprocal-cycle validation',
                  test_mode='friction-sweep', fits={}, quality={}, warnings=[
                      'J is neither fixed nor estimated in this near-steady experiment.',
                      'Intervals describe repeat variation within this capture, not systematic accuracy.',
                      'Current-derived feedback torque is not a torque sensor; torque scale must be calibrated.',
                      'Verify repeatability in a second capture; temperature/cables/nonlinear friction can change the result.'])
    if metadata['status'] != 2:
        report['warnings'].append(metadata['status_text'])
    coverage = []
    enough = True
    for n, speed in enumerate(speeds):
        record = dict(reference_rad_s=float(speed))
        for direction, label in [(1, 'positive'), (-1, 'negative')]:
            mask = selected & (level == n) & (np.sign(w) == direction)
            count, legs = int(mask.sum()), int(len(np.unique(leg[mask])))
            record[label] = dict(samples=count, traversals=legs,
                                 median_speed_rad_s=float(np.median(np.abs(w[mask]))) if count else None)
            enough &= count >= 4 and legs >= 2
        coverage.append(record)
    report['quality'] = dict(duration_s=float(a[-1, 0]), central_gate_rad=gate,
                             acceleration_gate_rad_s2=.4, selected_samples=int(selected.sum()),
                             speed_coverage=coverage)
    if not enough:
        report['warnings'].append('Insufficient steady traversals: need >=4 samples and >=2 traversals per direction at every speed. No b/fc result reported.')
        return report

    x = np.column_stack((w[selected], np.sign(w[selected]), q[selected], np.ones(selected.sum())))
    group = 2 * level[selected] + (w[selected] > 0).astype(int)
    cycles = cycle[selected]
    lo, hi = np.array([0., 0., -20., -1.]), np.array([5., .5, 20., 1.])

    def solve(design, response, groups):
        scale = np.linalg.norm(design, axis=0)
        if np.any(scale < 1e-10) or np.linalg.matrix_rank(design / scale) < 4:
            raise ValueError('Insufficient independent speed/position variation.')
        balance = 1 / np.bincount(groups, minlength=8)[groups]
        robust = np.ones(len(response))
        for _ in range(6):
            weight = np.sqrt(balance * robust)
            fit = lsq_linear(design / scale * weight[:, None], response * weight,
                             bounds=(lo * scale, hi * scale), tol=1e-11)
            p = fit.x / scale
            residual = response - design @ p
            sigma = max(.0005, 1.4826 * np.median(np.abs(residual - np.median(residual))))
            robust = np.minimum(1., 1.5 * sigma / np.maximum(np.abs(residual), 1e-12))
        return p

    for name, col in [('feedback', 5), ('command', 4)]:
        y = a[selected, col]
        p = solve(x, y, group)
        condition = float(np.linalg.cond(x / np.linalg.norm(x, axis=0)))
        result = dict(available=True, b_nm_s_rad=float(p[0]), coulomb_nm=float(p[1]),
                      stiffness_nuisance_nm_rad=float(p[2]), offset_nm=float(p[3]),
                      torque_rmse_nm=float(np.sqrt(np.mean((y - x @ p) ** 2))),
                      scaled_condition=condition,
                      parameter_at_bound=bool(np.any(np.isclose(p, lo, atol=1e-5)) or
                                              np.any(np.isclose(p, hi, atol=1e-5))))
        # Same-position opposite-direction half differences cancel offset and
        # check the speed-dependent resistance independently of the joint fit.
        pairs = []
        bin_id = np.floor((q[selected] + gate) / .006).astype(int)
        for n in range(4):
            for b in np.unique(bin_id):
                masks = [(level[selected] == n) & (bin_id == b) & (x[:, 1] == d) for d in (1, -1)]
                if any(m.sum() < 2 for m in masks):
                    continue
                pos, neg = (np.median(x[m], axis=0) for m in masks)
                tp, tn = (float(np.median(y[m])) for m in masks)
                pairs.append(dict(level=n, speed_rad_s=float((pos[0] - neg[0]) / 2),
                                  resistance_nm=float((tp - tn - p[2] * (pos[2] - neg[2])) / 2)))
        result['same_position_pairs'] = pairs
        if len(pairs) >= 8 and len(set(v['level'] for v in pairs)) >= 3:
            px = np.array([[r['speed_rad_s'], 1] for r in pairs])
            py = np.array([r['resistance_nm'] for r in pairs])
            pp = lsq_linear(px, py, bounds=(0, [5, .5])).x
            result['paired_cross_check'] = dict(b_nm_s_rad=float(pp[0]), coulomb_nm=float(pp[1]),
                                                torque_rmse_nm=float(np.sqrt(np.mean((px @ pp - py) ** 2))))
        validations = []
        for c in np.unique(cycles):
            keep = cycles != c
            if np.unique(group[keep]).size != 8:
                continue
            pred = solve(x[keep], y[keep], group[keep])
            validations.append(float(np.mean((x[~keep] @ pred - y[~keep]) ** 2)))
        result['leave_cycle_out_rmse_nm'] = float(np.sqrt(np.mean(validations))) if validations else None
        rng = np.random.default_rng(173)
        boot = []
        for _ in range(96):
            indices = []
            for n in range(4):
                pool = np.unique(cycles[level[selected] == n])
                for c in rng.choice(pool, len(pool), replace=True):
                    indices.extend(np.flatnonzero(cycles == c))
            ix = np.asarray(indices, dtype=int)
            boot.append(solve(x[ix], y[ix], group[ix])[:2])
        result['cycle_bootstrap_95pct'] = dict(zip(('b_nm_s_rad', 'coulomb_nm'),
                                                   np.percentile(boot, [2.5, 97.5], axis=0).T.tolist()))
        if condition > 50 or result['parameter_at_bound']:
            report['warnings'].append(name + ': ill-conditioned or at parameter bound; b/fc remain unconfirmed.')
        report['fits'][name] = result
    return report
