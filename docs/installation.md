# Installation

## Reference platform

Use Ubuntu 22.04 x86_64, Python 3.10, GCC 11 and CMake 3.22. The source uses C++17, OpenMP, Eigen and vendored pybind11 2.9.0. Python 3.11/3.12 support is not established for these bindings.

The local development dependency is **RaiSim 1.1.8**, from `raisimTech/raisimLib` revision `74b836fba1dbd3227ce61f8abf7bffbd8f6e41ca`. Pin that revision rather than following current RaiSim 2 installation instructions. The current upstream site describes different bindings and visualization.

## 1. System packages and source

```bash
sudo apt update
sudo apt install -y build-essential cmake git libeigen3-dev libvulkan1 libminizip1 ffmpeg

git clone --branch withDHALOneLeg https://github.com/HYUNSEOKKKKKK/Hopper_Nature.git ~/Hopper_Nature
cd ~/Hopper_Nature
mkdir -p .local/runtime
git clone https://github.com/raisimTech/raisimLib.git .local/runtime/raisimLib
git -C .local/runtime/raisimLib checkout 74b836fba1dbd3227ce61f8abf7bffbd8f6e41ca
```

Expected layout:

```text
~/Hopper_Nature/
├── setup.py
├── requirements.txt
├── raisimGymTorch/
└── .local/
    ├── env/                         # Optional shortcut to the Conda environment
    └── runtime/raisimLib/
        ├── raisim/linux/            # RaiSim 1.1.8 headers/libraries/CMake package
        ├── thirdParty/Eigen3/
        └── raisimUnity/             # Matching legacy viewer
```

The `.local/` directory is ignored by Git. Python is provided by the named Conda environment `hopper_nature`, outside the checkout. The build discovers the local SDK/Eigen prefixes; external SDK paths can be passed explicitly. See [workspace paths](workspace.md). The project directory name does not affect model or output paths.

## 2. RaiSim activation

Obtain your own **RaiSim 1.x-compatible** activation key through the [RaiSim licensing channel](https://raisim.com/). Place the supplied file at `~/.raisim/activation.raisim`; see the [pinned upstream activation instructions](https://github.com/raisimTech/raisimLib/blob/74b836fba1dbd3227ce61f8abf7bffbd8f6e41ca/docs/sections/Installation.rst).

The engine requires activation even when training without a viewer. License availability is determined by RaiSim; a fresh clone alone does not provide an engine license. Keep the key outside the repository.

## 3. Python environment

With an existing Miniforge/Conda installation:

```bash
conda create -n hopper_nature python=3.10 pip -y
conda activate hopper_nature
python -m pip install torch==2.5.1 --index-url https://download.pytorch.org/whl/cpu
python -m pip install -r requirements.txt
```

Alternatively, use `python3.10 -m venv .venv` and `source .venv/bin/activate` before the same pip and build commands. On a system without Python 3.10, install it first or use Conda. Recreate the environment after relocating its prefix.

For GPU training, replace the CPU PyTorch install command with an appropriate **2.5.1** wheel from the [official previous-version instructions](https://pytorch.org/get-started/previous-versions/). GPU execution is optional; the runner chooses CUDA when `torch.cuda.is_available()` is true. CUDA training has not been checked as part of this release preparation.

The Python requirements pin the directly used packages. PyTorch is separate because its CPU/CUDA wheels depend on the machine. This is not a complete transitive dependency lock; record `python -m pip freeze` with experiments intended for comparison.

## 4. Build the C++ extension

From the Hopper repository root, with the environment activated:

```bash
conda activate hopper_nature
python setup.py build_ext --inplace
python -c "from raisimGymTorch.env.bin import hopper; print(hopper.__file__)"
```

The build writes `raisimGymTorch/env/bin/hopper.cpython-310-*.so`. It discovers the SDK and Eigen under `.local/runtime/raisimLib/` and requires RaiSim 1.1.8 exactly. It does not need to build the upstream examples or install a global Python package. `runner.py` and `tester.py` support direct execution from the task directory without an editable installation or manual `PYTHONPATH` setting.

After changing `Environment.hpp`, rebuild with the same command. When switching Python interpreters or SDK locations, use a fresh build directory, for example `python setup.py build_ext --inplace --build-temp build/python310-clean`. An external SDK can be selected with `python setup.py build_ext --inplace --CMAKE_PREFIX_PATH 'RAISIM_PREFIX;EIGEN_PREFIX'`, using the actual prefixes.

Then run the [two-update check](training.md#first-run).

## 5. Optional visualization

The pinned RaiSim checkout contains both the engine and **RaiSim Unity** inside `.local/runtime/raisimLib/`. After building, the direct task commands can launch that client and wait for its connection to the simulation:

```bash
cd ~/Hopper_Nature/raisimGymTorch/env/envs/hopper
python runner.py --viewer --num-envs 4 --num-threads 2 --iterations 2 --no-tensorboard
# Or replay a complete checkpoint bundle:
python tester.py --viewer --weight /path/to/run/full_10000.pt
```

No global alias or external workspace is used. The client must exist at `.local/runtime/raisimLib/raisimUnity/linux/raisimUnity.x86_64`; missing files or a failed connection produce a diagnostic. The launched window closes with the command. `--viewer` and `--headless` are mutually exclusive. A desktop display is required; on remote servers use `--headless`.

For the original two-terminal workflow, omit `--viewer` and run `~/Hopper_Nature/.local/runtime/raisimLib/raisimUnity/linux/raisimUnity.x86_64` directly in another terminal. The default server port is 8080; this client uses `127.0.0.1` with automatic connection enabled in its `gui_settings.xml`.

Install the viewer's Linux dependencies if needed:

```bash
sudo apt install -y libvulkan1 libminizip1 ffmpeg
```

The pinned checkout also includes `raisimUnityOpengl/linux` for the older OpenGL client. Consult its [visualization documentation](https://github.com/raisimTech/raisimLib/blob/74b836fba1dbd3227ce61f8abf7bffbd8f6e41ca/docs/sections/Visualizers.rst) for client controls. The commands above use the Vulkan Unity client.

With rendering and `record_video: yes`, the runner records evaluation rollouts to `.local/runtime/raisimLib/raisimUnity/linux/Screenshot/` inside this checkout. Checkpoints remain in `raisimGymTorch/data/hopper/`. The tester does not start automatic recording. Unity startup/connection logs from `--viewer` are saved under `.local/logs/`. See the [local GUI/recording validation](validation.md) for checks actually performed.

## Troubleshooting

| Symptom | Resolution |
| --- | --- |
| CMake cannot find RaiSim | Pass the directory containing its `include/` and `lib/`, not this repository, in `--CMAKE_PREFIX_PATH`. |
| CMake cannot find Eigen | Include `raisimLib/thirdParty/Eigen3` in that prefix list, or install `libeigen3-dev`. |
| `No module named ...hopper` | Build with the active Python 3.10 interpreter and run from the repository root. |
| `libraisim.so.1.1.8` cannot be opened | Keep the dependency at its build-time location or rebuild with its new prefix. For a custom install, add its `lib/` to `LD_LIBRARY_PATH`. |
| Libraries load from an older workspace | Check inherited `LD_LIBRARY_PATH`. For the documented local SDK layout, use `unset LD_LIBRARY_PATH` in the current terminal before running; no global library path is needed. |
| Activation error before training | Check the engine version and your key at `~/.raisim/activation.raisim`. |
| `ruamel.yaml` dump/API error | Install the pinned 0.17.21 version; this source uses its older API. |
| High memory use or CPU contention | Start with `--num-envs 4 --num-threads 2`; increase only after the short check succeeds. |
| No viewer, remote host or no display | Use `--headless --no-tensorboard`; launch TensorBoard separately when useful. |
| `--viewer` reports missing Unity | Complete the pinned RaiSim clone/checkout step inside this repository; rebuild the extension after updating source. |
| Unity exits or does not connect | Read the printed `.local/logs/unity-*.log`; check display access, viewer libraries, and `gui_settings.xml` at `127.0.0.1:8080`. Only one simulation can use port 8080 at a time. |
| Checkpoint shape or missing CSV error | Use the checkpoint's own `cfg.yaml` and the mean/variance files from the same iteration. |

Fresh OS installation and successful activation on another machine remain outside the [local validation record](validation.md).
