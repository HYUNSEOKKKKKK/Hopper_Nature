"""Check project selection when another RaiSim checkout is in the parent shell."""

import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class WorkspaceLauncher(unittest.TestCase):
    def make_config(self, directory, version='1.1.8'):
        sdk = directory / 'sdk'
        cmake = sdk / 'lib/cmake/raisim'
        cmake.mkdir(parents=True)
        (cmake / 'raisimConfig-version.cmake').write_text(f'set(PACKAGE_VERSION "{version}")\n')
        (sdk / 'lib/libraisim.so.1.1.8').touch()
        eigen = directory / 'eigen'
        (eigen / 'share/eigen3/cmake').mkdir(parents=True)
        (eigen / 'share/eigen3/cmake/Eigen3Config.cmake').touch()
        config = directory / 'workspace.json'
        config.write_text(json.dumps({'python': sys.executable,
                                      'raisim_prefix': str(sdk), 'eigen_prefix': str(eigen)}))
        return config, sdk

    def invoke(self, config, *arguments):
        return subprocess.run(
            [sys.executable, '-B', '-S', str(ROOT / 'scripts/hopper.py'),
             '--config', str(config), *arguments],
            cwd=config.parent, env={**os.environ, 'PYTHONPATH': '/other/raisim2',
                                    'LD_LIBRARY_PATH': '/other/raisim2/lib'},
            text=True, capture_output=True, timeout=15)

    def test_foreign_cwd_and_raisim_paths_do_not_select_another_project(self):
        with tempfile.TemporaryDirectory() as name:
            config, sdk = self.make_config(Path(name))
            result = self.invoke(config, '--dry-run', 'train', '--num-envs', '4', '--headless')
            self.assertEqual(result.returncode, 0, result.stderr)
            planned = json.loads(result.stdout)
            self.assertEqual(planned['cwd'], str(ROOT))
            self.assertEqual(planned['environment']['PYTHONPATH'], str(ROOT))
            self.assertNotIn('/other/raisim2', planned['environment']['LD_LIBRARY_PATH'])
            self.assertEqual(planned['environment']['LD_LIBRARY_PATH'].split(':')[0], str(sdk / 'lib'))
            self.assertIn('raisimGymTorch.env.envs.hound.runner', planned['command'])
            self.assertEqual(planned['command'][-3:], ['--num-envs', '4', '--headless'])

    def test_rejects_raisim2_before_starting_python_or_simulation(self):
        with tempfile.TemporaryDirectory() as name:
            config, _ = self.make_config(Path(name), version='2.2.0')
            result = self.invoke(config, '--dry-run', 'replay')
            self.assertEqual(result.returncode, 2)
            self.assertIn('Expected RaiSim 1.1.8', result.stderr)

    def test_missing_explicit_config_has_a_clear_error(self):
        with tempfile.TemporaryDirectory() as name:
            result = self.invoke(Path(name) / 'missing.json', 'doctor')
            self.assertEqual(result.returncode, 2)
            self.assertIn('missing.json', result.stderr)
            self.assertNotIn('Traceback', result.stderr)
