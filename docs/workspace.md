# RaiSim 1 workspace and direct execution

Hopper Nature is a **RaiSim 1.1.8** research project. Source, simulator dependency and experiment records live in one project folder. Python is provided by the dedicated Conda environment **`hopper_nature`**, stored in Conda's standard environment directory. The personal activation key stays outside the project at `~/.raisim/activation.raisim`.

```text
Hopper_Nature/
├── setup.py                           # Build the native extension
├── README.md, RESUME.md, docs/
├── raisimGymTorch/
│   ├── env/envs/hound/
│   │   ├── runner.py                 # Run directly to train
│   │   ├── tester.py                 # Run directly to replay
│   │   ├── cfg.yaml
│   │   └── Environment.hpp
│   └── data/dhal_one_leg/<run>/       # Local training results, ignored by Git
├── rsc/                              # Robot model history and calibration
└── .local/                           # Workstation files, ignored by Git
    ├── runtime/raisimLib/             # RaiSim 1.1.8, Eigen and optional Unity viewer
    ├── checkpoints/<run>/            # Optional preserved result bundles
    ├── media/originals/              # Original recordings
    ├── environment/                  # Environment inventories
    ├── inventory/                    # Artifact/provenance and migration records
    ├── validation/                   # Build and execution logs
    └── archive/                      # Earlier local records and recovery material
```

The simulator and local research records are not included in a Git clone. Follow [installation](installation.md) to create the SDK layout and named Conda environment. The existing workstation also has `.local/env` as a compatibility shortcut to `hopper_nature`; it is not a second environment or a requirement for direct execution.

## Build and train in the original style

```bash
conda activate hopper_nature
cd ~/Hopper_Nature
python setup.py build_ext --inplace

cd raisimGymTorch/env/envs/hound
python runner.py
```

Rebuild from the repository root after changing C++ files such as `Environment.hpp`. `setup.py` selects the active Python, and CMake discovers RaiSim and Eigen under `.local/runtime/raisimLib/`. The engine must be version 1.1.8. The build writes `raisimGymTorch/env/bin/hound.cpython-310-*.so`.

`runner.py` and `tester.py` locate this checkout from their own file paths. Direct execution does not require `scripts/hopper.py`, an editable package installation or a manual `PYTHONPATH`. It is also possible to run `python raisimGymTorch/env/envs/hound/runner.py` from the root, or use the existing `python -m raisimGymTorch.env.envs.hound.runner` interface.

For a short check instead of full training, run from the task directory:

```bash
python runner.py --num-envs 4 --num-threads 2 --iterations 2 \
  --headless --no-tensorboard

python tester.py --weight /path/to/run/full_10000.pt \
  --headless --no-realtime
```

Training results go to `raisimGymTorch/data/dhal_one_leg/<timestamp>/`, regardless of the working directory. Relative user-supplied checkpoint and output paths are interpreted from the current working directory.

In a second terminal, start the matching legacy Unity viewer directly:

```bash
~/Hopper_Nature/.local/runtime/raisimLib/raisimUnity/linux/raisimUnity.x86_64
```

The project uses its own RaiSim dependency and viewer; the separate `raisim_ws` workspace is not needed by this build/run workflow. Adjust `~/Hopper_Nature` if the checkout root differs.

## Use an external SDK

With Python 3.10 activated, pass the actual dependency prefixes:

```bash
python setup.py build_ext --inplace --CMAKE_PREFIX_PATH \
  '/path/to/raisim/linux;/path/to/Eigen3'
```

After switching interpreters or SDKs, use a fresh build directory, for example `--build-temp build/python310-clean`. Native binaries should be rebuilt after moving to another machine. Conda environments also contain absolute prefix paths and should be recreated at their destination.

## Optional launcher compatibility

`scripts/hopper.py` remains available for existing commands, but is not required for the direct workflow above. Its path overrides live in the Git-ignored `.local/workspace.json`, for example:

```json
{
  "python": "/absolute/path/to/conda/envs/hopper_nature/bin/python",
  "raisim_prefix": ".local/runtime/raisimLib/raisim/linux",
  "eigen_prefix": ".local/runtime/raisimLib/thirdParty/Eigen3",
  "viewer": ".local/runtime/raisimLib/raisimUnity/linux/raisimUnity.x86_64"
}
```

Those settings affect the optional launcher only. Direct `setup.py` and `runner.py` commands use the active Python and the build prefixes described above.

## Preserve a complete experiment

Keep `full_N.pt`, `meanN.csv`, `varN.csv`, `cfg.yaml` and the associated `Environment.hpp` together under one run directory. Preserve run names when consolidating older records; resolve collisions before moving files. Robot revisions, calibration and original recordings are research inputs rather than build output.

Git tracks the source, model assets and public documentation. It does not back up `.local/` or training output directories. Store those separately when transferring or archiving the research. Environment inventories describe how to recreate the setup; a convenience copy on the same disk does not protect against losing that disk.

On the existing workstation, `.local/README.md` records actual paths, preserved checkpoints and compatibility links from earlier locations. Those links are migration aids and are not required by a fresh installation.
