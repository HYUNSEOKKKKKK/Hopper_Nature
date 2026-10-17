# One-folder RaiSim 1 workspace

Hopper Nature is a **RaiSim 1.1.8** research project. The recommended local workspace keeps its source, Python environment, simulator dependency and experiment records together. It does not share an editable `raisimGymTorch` installation with a RaiSim 2 project.

```text
Hopper_Nature/                         # This Git checkout
├── README.md, RESUME.md, docs/         # Public installation and research handoff
├── scripts/hopper.py                  # Workspace launcher
├── raisimGymTorch/
│   ├── env/envs/hound/                # Research configuration and C++ environment
│   └── data/dhal_one_leg/<run>/       # Local training results, ignored by Git
├── rsc/                              # Robot model history and calibration
└── .local/                           # Workstation files, ignored by Git
    ├── env/                          # Python 3.10 Conda prefix
    ├── runtime/raisimLib/             # RaiSim 1.1.8, Eigen and optional Unity viewer
    ├── workspace.json                # Optional path overrides
    ├── checkpoints/<run>/            # Optional preserved result bundles
    ├── media/originals/              # Original recordings
    ├── environment/                  # Environment inventories
    ├── inventory/                    # Artifact/provenance and migration records
    ├── validation/                   # Build and execution logs
    └── archive/                      # Earlier local records and recovery material
```

The environment and simulator directories are not included in a Git clone. Follow [installation](installation.md) to create them. The activation key stays at `~/.raisim/activation.raisim`, outside this workspace.

## Run one explicit workspace

```bash
cd ~/Hopper_Nature
python3 scripts/hopper.py build
python3 scripts/hopper.py doctor

python3 scripts/hopper.py train --num-envs 4 --num-threads 2 \
  --iterations 2 --headless --no-tensorboard

python3 scripts/hopper.py replay --weight /path/to/run/full_10000.pt \
  --headless --no-realtime
python3 scripts/hopper.py export --weight /path/to/run/full_10000.pt

# Optional legacy RaiSim 1 Unity client, in another terminal:
python3 scripts/hopper.py viewer
```

The launcher changes only its child process: it selects the configured Python, sets this checkout as the working directory and `PYTHONPATH`, and selects its RaiSim library and CMake prefixes. It checks the SDK's version file before execution and requires Python 3.10 for Python commands. An inherited RaiSim 2 `PYTHONPATH` or library path is replaced for this process. Shell startup files and other Conda environments do not need changes.

`doctor` reports the selected package, native extension, actual loaded RaiSim library and key-file presence. It does not activate a simulation, start a camera or establish policy performance. Use `--dry-run` before a command to inspect the selected paths:

```bash
python3 scripts/hopper.py --dry-run train --headless --no-tensorboard
```

The original `python -m raisimGymTorch.env.envs.hound.runner` and `tester` entry points remain available from the checkout root with the correct Python and runtime paths.

## Use an existing Python or SDK installation

Create `.local/workspace.json` with only the paths you need to override. Relative paths are resolved from the checkout root; absolute paths are also supported. For example:

```json
{
  "python": ".local/env/bin/python",
  "raisim_prefix": ".local/runtime/raisimLib/raisim/linux",
  "eigen_prefix": ".local/runtime/raisimLib/thirdParty/Eigen3",
  "viewer": ".local/runtime/raisimLib/raisimUnity/linux/raisimUnity.x86_64"
}
```

Use `--config /path/to/workspace.json` before the command for another configuration. The launcher does not install packages, download dependencies or modify an existing environment. It builds the native extension in `build/workspace-py310/` with the explicit SDK and Eigen prefixes. There is no need to run `pip install -e .` when using the launcher.

Choose the final workspace location before creating `.local/env`: Conda prefixes contain absolute paths. After moving a workspace to another machine or root, recreate its environment and rebuild the extension instead of treating the copied prefix or `.so` as portable.

## Preserve a complete experiment

Keep `full_N.pt`, `meanN.csv`, `varN.csv`, `cfg.yaml` and the associated `Environment.hpp` together under one run directory. Preserve run names when consolidating older records; resolve any collisions before moving files. Robot revisions, calibration and original recordings are research inputs rather than build output.

Git tracks the source, model assets and public documentation. It does not back up `.local/` or training output directories. Store those separately when transferring or archiving the research. Environment inventories describe how to recreate the setup, while native builds should be regenerated. A convenience copy on the same disk does not protect against losing that disk.

On an existing workstation, `.local/README.md` records the actual local paths, preserved checkpoints and compatibility links from earlier locations. Compatibility links are local migration aids and are not required by a fresh installation.
