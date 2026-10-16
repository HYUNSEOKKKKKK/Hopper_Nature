# Installation

## Reference platform

Use Ubuntu 22.04 x86_64, Python 3.10, GCC 11 and CMake 3.22. The source uses C++17, OpenMP, Eigen and vendored pybind11 2.9.0. Python 3.11/3.12 support is not established for these bindings.

The local development dependency is **RaiSim 1.1.8**, from `raisimTech/raisimLib` revision `74b836fba1dbd3227ce61f8abf7bffbd8f6e41ca`. Pin that revision rather than following current RaiSim 2 installation instructions. The current upstream site describes different bindings and visualization.

## 1. System packages and source

```bash
sudo apt update
sudo apt install -y build-essential cmake git libeigen3-dev

mkdir -p ~/hopper_ws
cd ~/hopper_ws
git clone https://github.com/raisimTech/raisimLib.git
git -C raisimLib checkout 74b836fba1dbd3227ce61f8abf7bffbd8f6e41ca
git clone --branch withDHALOneLeg https://github.com/HYUNSEOKKKKKK/Hopper_Nature.git
cd Hopper_Nature
```

Expected layout:

```text
~/hopper_ws/
├── raisimLib/
│   ├── raisim/linux/       # RaiSim headers, libraries and CMake package
│   ├── thirdParty/Eigen3/
│   └── raisimUnity/        # Legacy viewer distributed with that revision
└── Hopper_Nature/
```

An existing layout with Hopper inside `raisimLib/` also works; pass the actual RaiSim and Eigen prefix paths at build time. The project directory name does not affect model or output paths.

## 2. RaiSim activation

Obtain your own **RaiSim 1.x-compatible** activation key through the [RaiSim licensing channel](https://raisim.com/). Place the supplied file at `~/.raisim/activation.raisim`; see the [pinned upstream activation instructions](https://github.com/raisimTech/raisimLib/blob/74b836fba1dbd3227ce61f8abf7bffbd8f6e41ca/docs/sections/Installation.rst).

The engine requires activation even when training without a viewer. License availability is determined by RaiSim; a fresh clone alone does not provide an engine license. Keep the key outside the repository.

## 3. Python environment

With an existing Miniforge/Conda installation:

```bash
conda create -n hopper python=3.10 pip -y
conda activate hopper
python -m pip install torch==2.5.1 --index-url https://download.pytorch.org/whl/cpu
python -m pip install -r requirements.txt
```

Alternatively, use `python3.10 -m venv .venv` and `source .venv/bin/activate` before the same pip commands. On a system without Python 3.10, install it first or use Conda.

For GPU training, replace the CPU PyTorch install command with an appropriate **2.5.1** wheel from the [official previous-version instructions](https://pytorch.org/get-started/previous-versions/). GPU execution is optional; the runner chooses CUDA when `torch.cuda.is_available()` is true. CUDA training has not been checked as part of this release preparation.

The Python requirements pin the directly used packages. PyTorch is separate because its CPU/CUDA wheels depend on the machine. This is not a complete transitive dependency lock; record `python -m pip freeze` with experiments intended for comparison.

## 4. Build the C++ extension

From the Hopper repository root, with the Python environment activated:

```bash
python setup.py build_ext --inplace --CMAKE_PREFIX_PATH \
  "$PWD/../raisimLib/raisim/linux;$PWD/../raisimLib/thirdParty/Eigen3"

python -c "from raisimGymTorch.env.bin import hound; print(hound.__file__)"
```

The build writes `raisimGymTorch/env/bin/hound.cpython-310-*.so`. It does not need to build the upstream RaiSim examples or install a global Python package. Use `python -m ...` from the repository root so this checkout is imported.

After changing `Environment.hpp`, rebuild with the same command. When switching Python interpreters, use a clean build directory, for example `python setup.py build_ext --inplace --build-temp build/python310-clean --CMAKE_PREFIX_PATH '...'` with the real prefixes substituted.

Then run the [two-update check](training.md#first-run).

## 5. Optional visualization

This environment creates a legacy `RaisimServer` when `render` is true. Use the **RaiSim Unity** client from the pinned RaiSim 1.x checkout and connect to the local server (default TCP port 8080). Run the `linux/raisimUnity.x86_64` executable inside that checkout's `raisimUnity` directory. Start the simulation and client in separate terminals.

Install the viewer's Linux dependencies if needed:

```bash
sudo apt install -y libvulkan1 libminizip1 ffmpeg
```

The pinned checkout also includes `raisimUnityOpengl/linux` for the older OpenGL client. Consult its [visualization documentation](https://github.com/raisimTech/raisimLib/blob/74b836fba1dbd3227ce61f8abf7bffbd8f6e41ca/docs/sections/Visualizers.rst) for client controls. Current RaiSim 2 viewers are not the documented workflow here. Video recording depends on a connected compatible client and has not been validated by the headless checks.

## Troubleshooting

| Symptom | Resolution |
| --- | --- |
| CMake cannot find RaiSim | Pass the directory containing its `include/` and `lib/`, not this repository, in `--CMAKE_PREFIX_PATH`. |
| CMake cannot find Eigen | Include `raisimLib/thirdParty/Eigen3` in that prefix list, or install `libeigen3-dev`. |
| `No module named ...hound` | Build with the active Python 3.10 interpreter and run from the repository root. |
| `libraisim.so.1.1.8` cannot be opened | Keep the dependency at its build-time location or rebuild with its new prefix. For a custom install, add its `lib/` to `LD_LIBRARY_PATH`. |
| Activation error before training | Check the engine version and your key at `~/.raisim/activation.raisim`. |
| `ruamel.yaml` dump/API error | Install the pinned 0.17.21 version; this source uses its older API. |
| High memory use or CPU contention | Start with `--num-envs 4 --num-threads 2`; increase only after the short check succeeds. |
| No viewer, remote host or no display | Use `--headless --no-tensorboard`; launch TensorBoard separately when useful. |
| Checkpoint shape or missing CSV error | Use the checkpoint's own `cfg.yaml` and the mean/variance files from the same iteration. |

Fresh OS installation and successful activation on another machine remain outside the [local validation record](validation.md).
