"""Run Hopper with its own Python environment and RaiSim 1.1.8 dependencies."""

import argparse
import json
import os
from pathlib import Path
import re
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = ROOT / '.local/workspace.json'
DEFAULTS = {
    'python': '.local/env/bin/python',
    'raisim_prefix': '.local/runtime/raisimLib/raisim/linux',
    'eigen_prefix': '.local/runtime/raisimLib/thirdParty/Eigen3',
    'viewer': '.local/runtime/raisimLib/raisimUnity/linux/raisimUnity.x86_64',
}

DOCTOR = r'''
import importlib.metadata
import importlib.util
import json
from pathlib import Path
import sys

root = Path(sys.argv[1]).resolve()
sdk = Path(sys.argv[2]).resolve()
if sys.version_info[:2] != (3, 10):
    raise SystemExit('Hopper requires Python 3.10; selected: ' + sys.version)
spec = importlib.util.find_spec('raisimGymTorch')
expected = root / 'raisimGymTorch/__init__.py'
if spec is None or Path(spec.origin).resolve() != expected:
    raise SystemExit('Wrong raisimGymTorch checkout: ' + str(spec))
torch_version = importlib.metadata.version('torch')
if torch_version.split('+')[0].split('.')[:3] != ['2', '5', '1']:
    raise SystemExit('Expected PyTorch 2.5.1 series; selected: ' + torch_version)
yaml_version = importlib.metadata.version('ruamel.yaml')
if yaml_version != '0.17.21':
    raise SystemExit('Expected ruamel.yaml 0.17.21; selected: ' + yaml_version)
from raisimGymTorch.env.bin import hound
native = Path(hound.__file__).resolve()
if native.parent != root / 'raisimGymTorch/env/bin':
    raise SystemExit('Wrong native extension: ' + str(native))
loaded = sorted({line.split()[-1] for line in Path('/proc/self/maps').read_text().splitlines()
                 if 'libraisim.so.' in line})
expected_library = (sdk / 'lib/libraisim.so.1.1.8').resolve()
if not loaded or any(Path(item).resolve() != expected_library for item in loaded):
    raise SystemExit('Unexpected loaded RaiSim library: ' + str(loaded))
print(json.dumps({
    'python': sys.executable,
    'python_version': sys.version.split()[0],
    'torch': torch_version,
    'ruamel.yaml': yaml_version,
    'package': str(expected),
    'native_extension': str(native),
    'raisim_library': str(expected_library),
    'activation_key_present': (Path.home() / '.raisim/activation.raisim').is_file(),
    'scope': 'Package and native-library imports only; no simulation or hardware started.',
}, indent=2))
'''


def settings_from(config):
    values = DEFAULTS.copy()
    path = config if config is not None else DEFAULT_CONFIG
    if config is not None or path.exists():
        supplied = json.loads(path.read_text())
        if not isinstance(supplied, dict) or supplied.keys() - values.keys():
            raise ValueError('Workspace configuration must contain only: ' + ', '.join(values))
        values.update(supplied)
    paths = {}
    for key, value in values.items():
        if not isinstance(value, str) or not value.strip():
            raise ValueError('Invalid workspace path: ' + key)
        path = Path(value).expanduser()
        paths[key] = (path if path.is_absolute() else ROOT / path).resolve()
    return paths


def validate_paths(paths):
    if not paths['python'].is_file():
        raise ValueError('Python environment not found: ' + str(paths['python']) +
                         '; follow docs/workspace.md')
    version = paths['raisim_prefix'] / 'lib/cmake/raisim/raisimConfig-version.cmake'
    if not version.is_file() or not re.search(
            r'set\(PACKAGE_VERSION\s+"1\.1\.8"\)', version.read_text()):
        raise ValueError('Expected RaiSim 1.1.8 at: ' + str(paths['raisim_prefix']))
    if not (paths['raisim_prefix'] / 'lib/libraisim.so.1.1.8').is_file():
        raise ValueError('RaiSim 1.1.8 Linux library is missing')
    if not (paths['eigen_prefix'] / 'share/eigen3/cmake/Eigen3Config.cmake').is_file():
        raise ValueError('Eigen CMake package not found at: ' + str(paths['eigen_prefix']))


def child_environment(paths):
    env = os.environ.copy()
    prefix = paths['python'].parent.parent
    env.pop('PYTHONHOME', None)
    env.pop('VIRTUAL_ENV', None)
    env.update({
        'PYTHONPATH': str(ROOT),
        'PYTHONNOUSERSITE': '1',
        'PYTHONDONTWRITEBYTECODE': '1',
        'LD_LIBRARY_PATH': str(paths['raisim_prefix'] / 'lib') + ':' + str(prefix / 'lib'),
        'CMAKE_PREFIX_PATH': ':'.join(str(paths[k]) for k in ('raisim_prefix', 'eigen_prefix')),
        'RAISIM_DIR': str(paths['raisim_prefix']),
        'WORKSPACE': str(ROOT),
        'CONDA_PREFIX': str(prefix),
        'CONDA_DEFAULT_ENV': str(prefix),
        'PATH': str(prefix / 'bin') + os.pathsep + env.get('PATH', os.defpath),
    })
    return env


def command_for(command, arguments, paths):
    python = [str(paths['python']), '-B']
    if command == 'doctor':
        if arguments:
            raise ValueError('doctor takes no additional arguments')
        return python + ['-c', DOCTOR, str(ROOT), str(paths['raisim_prefix'])]
    if command == 'build':
        return python + [str(ROOT / 'setup.py'), 'build_ext', '--inplace',
                         '--build-temp', str(ROOT / 'build/workspace-py310'),
                         '--CMAKE_PREFIX_PATH',
                         str(paths['raisim_prefix']) + ';' + str(paths['eigen_prefix'])] + arguments
    if command in ('train', 'replay'):
        module = 'runner' if command == 'train' else 'tester'
        return python + ['-m', 'raisimGymTorch.env.envs.hound.' + module] + arguments
    if command == 'export':
        return python + [str(ROOT / 'scripts/export_policy.py')] + arguments
    if command == 'viewer':
        if not paths['viewer'].is_file():
            raise ValueError('Legacy RaiSim Unity viewer not found: ' + str(paths['viewer']))
        return [str(paths['viewer'])] + arguments
    return python + arguments


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, help='path overrides; default: .local/workspace.json')
    parser.add_argument('--dry-run', action='store_true', help='print the command and scoped paths')
    parser.add_argument('command', choices=('doctor', 'build', 'train', 'replay', 'export', 'viewer', 'python'))
    parser.add_argument('arguments', nargs=argparse.REMAINDER,
                        help='arguments passed to the selected command')
    args = parser.parse_args()
    try:
        paths = settings_from(args.config)
        validate_paths(paths)
        arguments = args.arguments[1:] if args.arguments[:1] == ['--'] else args.arguments
        command = command_for(args.command, arguments, paths)
        env = child_environment(paths)
    except (OSError, ValueError) as error:
        parser.error(str(error))
    if args.dry_run:
        print(json.dumps({'cwd': str(ROOT), 'command': command,
                          'environment': {key: env[key] for key in (
                              'PYTHONPATH', 'PYTHONNOUSERSITE', 'LD_LIBRARY_PATH',
                              'CMAKE_PREFIX_PATH', 'RAISIM_DIR', 'CONDA_PREFIX')}}, indent=2))
        return 0
    if args.command != 'viewer':
        check = subprocess.run([str(paths['python']), '-B', '-c',
                                'import sys; sys.exit(0 if sys.version_info[:2] == (3, 10) else 1)'],
                               cwd=ROOT, env=env)
        if check.returncode:
            parser.error('The selected environment must use Python 3.10')
    try:
        return subprocess.run(command, cwd=ROOT, env=env).returncode
    except OSError as error:
        parser.error(str(error))


if __name__ == '__main__':
    raise SystemExit(main())
