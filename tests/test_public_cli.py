"""Exercise public entry points before installing simulator/learning dependencies."""

from pathlib import Path
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
            ['-m', 'raisimGymTorch.env.envs.hound.runner'],
            ['-m', 'raisimGymTorch.env.envs.hound.tester'],
            ['scripts/export_policy.py'],
        ):
            with self.subTest(command=command):
                result = self.run_cli(*command, '--help')
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertIn('usage:', result.stdout)

    def test_invalid_training_size_fails_before_importing_dependencies(self):
        result = self.run_cli('-m', 'raisimGymTorch.env.envs.hound.runner', '--num-envs', '0')
        self.assertEqual(result.returncode, 2)
        self.assertIn('must be at least 1', result.stderr)
        self.assertNotIn('ModuleNotFoundError', result.stderr)

    def test_replay_and_export_reject_incomplete_result_bundle(self):
        with tempfile.TemporaryDirectory() as directory:
            checkpoint = Path(directory) / 'full_1000.pt'
            checkpoint.touch()
            for command in (
                ['-m', 'raisimGymTorch.env.envs.hound.tester'],
                ['scripts/export_policy.py'],
            ):
                with self.subTest(command=command):
                    result = self.run_cli(*command, '--weight', str(checkpoint))
                    self.assertEqual(result.returncode, 2)
                    self.assertIn('mean1000.csv', result.stderr)
                    self.assertNotIn('ModuleNotFoundError', result.stderr)

    def test_replay_requires_explicit_checkpoint(self):
        result = self.run_cli('-m', 'raisimGymTorch.env.envs.hound.tester')
        self.assertEqual(result.returncode, 2)
        self.assertIn('--weight', result.stderr)


if __name__ == '__main__':
    unittest.main()
