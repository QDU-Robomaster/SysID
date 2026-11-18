"""Compile host-only tests and fit temporary synthetic captures; no hardware."""
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

root = Path(__file__).resolve().parents[1]
compiler = shutil.which('g++')
if compiler is None:
    raise SystemExit('g++ is required on PATH for host tests.')

with tempfile.TemporaryDirectory(prefix='sysid-test-') as folder:
    output = Path(folder)
    for name, source, extra in (
        ('sweep', 'friction_sweep_test.cpp', []),
        ('core', 'motor_sysid_test.cpp', []),
        ('adapter', 'motor_identifier_test.cpp', ['-I', str(root/'tests/stubs')]),
        ('entry', 'sysid_entry_test.cpp', ['-I', str(root/'tests/stubs')]),
        ('restored', 'restored_equivalence_test.cpp', []),
        ('sampling', 'imu_sampling_test.cpp', ['-I', str(root/'tests/stubs')]),
    ):
        binary = output/(name + ('.exe' if os.name == 'nt' else ''))
        subprocess.run([compiler, '-std=c++17', '-O2', '-Wall', '-Wextra',
                        '-Werror', *extra, '-I', str(root),
                        str(root/'tests'/source), '-o', str(binary)], check=True)
        subprocess.run([str(binary), str(output)], check=True)
    subprocess.run([sys.executable, str(root/'tests/friction_fit_test.py'), str(output)], check=True)
    subprocess.run([sys.executable, str(root/'tests/test_identify.py'),
                    str(output)], check=True)
subprocess.run([sys.executable,str(root/'tests/noisy_interval_fit_test.py')],check=True)
print('All host tests passed. No hardware was accessed.')
