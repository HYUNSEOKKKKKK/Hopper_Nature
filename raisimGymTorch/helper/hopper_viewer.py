"""Start the Unity client installed inside this checkout."""

import atexit
from datetime import datetime
import os
import subprocess
import time

from raisimGymTorch.helper.hopper_config import REPO_ROOT


def viewer_argument(parser):
    group = parser.add_mutually_exclusive_group()
    group.add_argument('--headless', action='store_true', help='disable visualization and video')
    group.add_argument('--viewer', action='store_true',
                       help='start this checkout\'s RaiSim Unity and wait for its connection')


def check_viewer(parser, enabled):
    if not enabled:
        return None
    executable = REPO_ROOT / '.local/runtime/raisimLib/raisimUnity/linux/raisimUnity.x86_64'
    if not executable.is_file() or not os.access(executable, os.X_OK):
        parser.error(f'local Unity executable missing: {executable}; '
                     'install the pinned RaiSim dependency as described in README.md')
    if not os.environ.get('DISPLAY'):
        parser.error('--viewer needs a graphical desktop (DISPLAY); use --headless on a server')
    return executable


def stop_viewer(process):
    if process is None or process.poll() is not None:
        return
    process.terminate()
    try:
        process.wait(timeout=10)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait()


def start_viewer(env, executable):
    if executable is None:
        return None
    if env.is_viewer_connected():
        raise RuntimeError('A Unity client is already connected. Close it before using --viewer, '
                           'or omit --viewer to use the manually started client.')
    (executable.parent / 'Screenshot').mkdir(exist_ok=True)
    logs = REPO_ROOT / '.local/logs'
    logs.mkdir(parents=True, exist_ok=True)
    log = logs / f'unity-{datetime.now():%Y-%m-%d-%H-%M-%S}-{os.getpid()}.log'
    console_log = log.with_suffix('.console.log')
    environment = os.environ.copy()
    # Unity and FFmpeg use their own/system libraries, without another project's SDK
    # or the training Conda environment's shared-library overrides.
    environment.pop('LD_LIBRARY_PATH', None)
    environment.pop('LD_PRELOAD', None)
    environment.pop('PYTHONPATH', None)
    try:
        with console_log.open('ab') as output:
            process = subprocess.Popen(
                [str(executable), '-logFile', str(log)], cwd=executable.parent,
                env=environment, stdout=output, stderr=subprocess.STDOUT,
                start_new_session=True)
    except OSError as error:
        raise RuntimeError(f'Cannot start local Unity: {error}') from error
    atexit.register(stop_viewer, process)
    print('Unity executable:', executable, flush=True)
    print('Unity log:', log, flush=True)
    print('Unity recordings:', executable.parent / 'Screenshot', flush=True)
    env.turn_on_visualization()
    deadline = time.monotonic() + 30
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError(f'Unity exited with code {process.returncode}; '
                               f'see {log} and {console_log}')
        if env.is_viewer_connected():
            print('Local Unity connected to the simulation.', flush=True)
            return process
        time.sleep(0.1)
    stop_viewer(process)
    raise RuntimeError(f'Unity did not connect within 30 seconds; see {log} and {console_log}. '
                       'Check the display and gui_settings.xml (127.0.0.1:8080).')
