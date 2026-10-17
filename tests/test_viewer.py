"""Check that Unity launch never falls back to another workspace."""

import argparse
from contextlib import redirect_stderr, redirect_stdout
import io
import json
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

from raisimGymTorch.helper import hopper_viewer


class LocalViewer(unittest.TestCase):
    def test_existing_connection_is_not_used_for_automatic_local_recording(self):
        class ConnectedSimulation:
            def is_viewer_connected(self):
                return True

        with patch.object(hopper_viewer.subprocess, 'Popen') as launch:
            with self.assertRaisesRegex(RuntimeError, 'already connected'):
                hopper_viewer.start_viewer(ConnectedSimulation(), Path('/unused/raisimUnity.x86_64'))
            launch.assert_not_called()

    def test_missing_local_viewer_does_not_use_legacy_viewer_on_path(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            legacy = root / 'legacy'
            legacy.mkdir()
            (legacy / 'raisimUnity').write_text('#!/bin/sh\nexit 0\n')
            (legacy / 'raisimUnity').chmod(0o755)
            message = io.StringIO()
            with patch.object(hopper_viewer, 'REPO_ROOT', root / 'checkout'), \
                    patch.dict(os.environ, {'DISPLAY': ':0', 'PATH': str(legacy)}), \
                    redirect_stderr(message), self.assertRaises(SystemExit) as error:
                hopper_viewer.check_viewer(argparse.ArgumentParser(), True)
            self.assertEqual(error.exception.code, 2)
            self.assertIn('local Unity executable missing', message.getvalue())
            self.assertIn('README.md', message.getvalue())

    def test_launch_uses_local_executable_and_cleans_inherited_library_paths(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            executable = root / '.local/runtime/raisimLib/raisimUnity/linux/raisimUnity.x86_64'
            executable.parent.mkdir(parents=True)
            executable.write_text(f'#!{sys.executable}\n' + '''import json, os, pathlib, sys, time
target = pathlib.Path(sys.argv[2])
target.with_suffix('.tmp').write_text(json.dumps({
    'cwd': os.getcwd(),
    'LD_LIBRARY_PATH': os.environ.get('LD_LIBRARY_PATH'),
    'PYTHONPATH': os.environ.get('PYTHONPATH'),
}))
target.with_suffix('.tmp').replace(target)
time.sleep(30)
''')
            executable.chmod(0o755)

            class Simulation:
                def turn_on_visualization(self):
                    pass

                def is_viewer_connected(self):
                    return any(path.stat().st_size for path in (root / '.local/logs').glob('*.log'))

            process = None
            try:
                with patch.object(hopper_viewer, 'REPO_ROOT', root), \
                        patch.dict(os.environ, {'LD_LIBRARY_PATH': '/other/raisim/lib',
                                                'PYTHONPATH': '/other/project'}), \
                        redirect_stdout(io.StringIO()):
                    process = hopper_viewer.start_viewer(Simulation(), executable)
                result = json.loads(next(path for path in (root / '.local/logs').glob('*.log')
                                         if path.stat().st_size).read_text())
                self.assertEqual(result['cwd'], str(executable.parent))
                self.assertIsNone(result['LD_LIBRARY_PATH'])
                self.assertIsNone(result['PYTHONPATH'])
            finally:
                hopper_viewer.stop_viewer(process)
            self.assertIsNotNone(process.poll())


if __name__ == '__main__':
    unittest.main()
