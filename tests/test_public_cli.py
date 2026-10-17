"""Exercise public entry points before installing simulator/learning dependencies."""

from pathlib import Path
import os
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class PublicCLI(unittest.TestCase):
    def run_cli(self, *args):
        return subprocess.run([sys.executable, '-S', *args], cwd=ROOT,
                              text=True, capture_output=True, timeout=15)

    def test_help_without_site_packages_or_native_build(self):
        for command in (
            ['-m', 'raisimGymTorch.env.envs.hopper.runner'],
            ['-m', 'raisimGymTorch.env.envs.hopper.tester'],
            ['-m', 'raisimGymTorch.env.envs.hopper.export'],
        ):
            with self.subTest(command=command):
                result = self.run_cli(*command, '--help')
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertIn('usage:', result.stdout)

    def test_task_files_run_directly_without_package_install_or_pythonpath(self):
        task = ROOT / 'raisimGymTorch/env/envs/hopper'
        environment = os.environ.copy()
        environment.pop('PYTHONPATH', None)
        with tempfile.TemporaryDirectory() as directory:
            for name in ('runner.py', 'tester.py', 'export.py'):
                for cwd in (task, Path(directory)):
                    with self.subTest(name=name, cwd=cwd):
                        result = subprocess.run(
                            [sys.executable, '-B', '-S', str(task / name), '--help'],
                            cwd=cwd, env=environment, text=True, capture_output=True, timeout=15)
                        self.assertEqual(result.returncode, 0, result.stderr)
                        self.assertIn('usage:', result.stdout)

    def test_invalid_training_size_fails_before_importing_dependencies(self):
        result = self.run_cli('-m', 'raisimGymTorch.env.envs.hopper.runner', '--num-envs', '0')
        self.assertEqual(result.returncode, 2)
        self.assertIn('must be at least 1', result.stderr)
        self.assertNotIn('ModuleNotFoundError', result.stderr)

    def test_replay_and_export_reject_incomplete_result_bundle(self):
        with tempfile.TemporaryDirectory() as directory:
            checkpoint = Path(directory) / 'full_1000.pt'
            checkpoint.touch()
            for command in (
                ['-m', 'raisimGymTorch.env.envs.hopper.tester'],
                ['-m', 'raisimGymTorch.env.envs.hopper.export'],
            ):
                with self.subTest(command=command):
                    result = self.run_cli(*command, '--weight', str(checkpoint))
                    self.assertEqual(result.returncode, 2)
                    self.assertIn('mean1000.csv', result.stderr)
                    self.assertNotIn('ModuleNotFoundError', result.stderr)

    def test_replay_requires_explicit_checkpoint(self):
        result = self.run_cli('-m', 'raisimGymTorch.env.envs.hopper.tester')
        self.assertEqual(result.returncode, 2)
        self.assertIn('--weight', result.stderr)

    def test_viewer_and_headless_are_mutually_exclusive(self):
        for module in ('runner', 'tester'):
            with self.subTest(module=module):
                result = self.run_cli('-m', f'raisimGymTorch.env.envs.hopper.{module}',
                                      '--viewer', '--headless')
                self.assertEqual(result.returncode, 2)
                self.assertIn('not allowed with argument', result.stderr)
                self.assertNotIn('ModuleNotFoundError', result.stderr)


if __name__ == '__main__':
    unittest.main()
