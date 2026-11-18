"""Decode an actual C++ sweep fixture and verify independent friction recovery."""
import sys
import struct
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from identify import decode
from friction_fit import fit_friction

raw = (Path(sys.argv[1]) / 'friction.bin').read_bytes()
rows, meta = decode(raw)
assert len(raw) == 0x12080 and meta['status'] == 2
assert meta['test_mode'] == 'friction-sweep' and meta['sweep_repeats'] == 3
assert {r[10] for r in rows} == {0, 5, 6, 7, 10, 11, 12, 13}
rng = np.random.default_rng(21)
a = np.array(rows, dtype=float)
# Noise is added independently of the plant/controller. The fixture was
# simulated with J=.006, b=.08, fc=.03, k=1.2, offset=.008.
a[1:, 2] += rng.normal(0, .002, len(a) - 1)
a[1:, 5] += rng.normal(0, .0006, len(a) - 1)
fit = fit_friction(a.tolist(), meta)
for source in ('command', 'feedback'):
    f = fit['fits'][source]
    assert not f['parameter_at_bound'], f
    assert abs(f['b_nm_s_rad'] / .08 - 1) < .10, f
    assert abs(f['coulomb_nm'] / .03 - 1) < .05, f
    assert f['leave_cycle_out_rmse_nm'] < .003, f
    assert len(f['cycle_bootstrap_95pct']['b_nm_s_rad']) == 2
    assert f['paired_cross_check']['b_nm_s_rad'] > .06
# Narrow/one-sided/sticking data must not produce a deceptively precise fit.
for mask in (a[:, 10] != 13, a[:, 2] >= 0):
    subset = np.concatenate((a[:1], a[1:][mask[1:]]))
    assert not fit_friction(subset.tolist(), meta)['fits']
bad = bytearray(raw)
struct.pack_into('<I', bad, 28, 2)
try:
    decode(bad)
except ValueError:
    pass
else:
    raise AssertionError('Unknown mode flags accepted.')
print('PASS: four-speed b/fc recovery with noise, cycle validation, rejected insufficient data.')
