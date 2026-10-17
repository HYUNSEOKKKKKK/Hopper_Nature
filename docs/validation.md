# Validation record

Initial release check: **2026-10-16**. Workspace consolidation check: **2026-10-17**. Local Unity/recording check: **2026-10-18**. These checks cover release tooling and local simulation execution. They do not report trained policy or hardware performance.

## Environment

| Item | Observed value |
| --- | --- |
| OS | Ubuntu 22.04, Linux x86_64, kernel 6.8 |
| Compiler / CMake | GCC 11.4.0 / CMake 3.22.1 |
| Python | 3.10.19 |
| PyTorch | Existing Conda `2.5.1.post303`; earlier checks selected CPU |
| NumPy / ruamel.yaml | 2.2.6 / 0.17.21 |
| TensorBoard / AdamP | 2.20.0 / 0.3.0 |
| RaiSim | 1.1.8 from upstream revision `74b836fba1dbd3227ce61f8abf7bffbd8f6e41ca`, with an existing local key |
| Environment source | Committed `Environment.hpp` at `d7d995813c121053eb5126634cafd45b3f3909cd` |

The native build and simulation checks used a separate source export with the environment revision listed above and the public Python entry points. The extension was rebuilt there, rather than reusing an existing development binary.

The installed RaiSim headers/libraries, Eigen and Unity assets were compared against the pinned public upstream revision and had no Git-tracked differences. The dependency pin therefore points to an upstream commit available to other users.

## Checks

| Check | Result |
| --- | --- |
| Native C++ extension build | Passed with Python 3.10 and explicit RaiSim/Eigen prefixes |
| CPU training | Passed: 4 environments, 2 threads, 2 PPO updates, 3200 training transitions |
| Saved run configuration | Captured effective CLI overrides, `Environment.hpp`, checkpoint and observation CSVs |
| Headless replay | Passed: 22 simulated seconds / 2200 control steps, all command phases exercised |
| Policy export | Actor, estimator and observation CSVs produced |
| Legacy export comparison | Actor/estimator text identical after trimming the final newline; CSVs identical |
| CLI preflight tests | 4 unittest cases passed, with site packages disabled in CLI subprocesses |
| Documentation/source checks | Relative file links, public Python syntax, active URDF mesh references and 17.100 kg model mass checked by `tests/test_repository.py` |

Replay used the smoke run's **untrained** `full_0.pt`. It had 14 episode terminations with automatic resets. Completion verifies execution, configuration loading and the command schedule, not successful continuous locomotion. Reward and timing from this short run are not presented as benchmark results.

## Reproduce the local check

After installation, run:

```bash
python -m unittest discover -s tests -v

python -m raisimGymTorch.env.envs.hopper.runner \
  --num-envs 4 --num-threads 2 --iterations 2 \
  --headless --no-tensorboard

# Substitute the actual timestamp printed by the runner:
python -m raisimGymTorch.env.envs.hopper.tester \
  --weight raisimGymTorch/data/hopper/<timestamp>/full_0.pt \
  --duration 22 --headless --no-realtime

python raisimGymTorch/env/envs/hopper/export.py \
  --weight raisimGymTorch/data/hopper/<timestamp>/full_0.pt
```

Replace `<timestamp>` before executing the commands. GitHub Actions runs only the checks that do not need a licensed simulator.

## Workstation handoff check — 2026-10-16

The current native extension was rebuilt during local cleanup, including the final environment settings committed in `029f4d5`. The existing local `2026-10-10-21-43-35/full_10000.pt` checkpoint was copied with its configuration, environment source and matching normalization files into the workstation's Git-excluded handoff folder. Its saved `Environment.hpp` matched the current source byte for byte.

A headless replay completed **22 simulated seconds / 2200 steps with zero episode terminations** for that one rollout. The documentation checks and four CLI unittest cases also passed. This check establishes that the preserved bundle can be loaded and run on the original workstation; it does not establish tracking accuracy, robustness or a recording-to-checkpoint association. The earlier smoke-run results above used a different, untrained checkpoint and remain a separate record. Local build/replay logs and dependency snapshots are indexed in [RESUME.md](../RESUME.md).

## One-folder workspace check — 2026-10-17

The project, its historical training runs, a dedicated Python 3.10 Conda prefix and the local RaiSim 1.1.8/Eigen/Unity dependencies were consolidated into one workstation directory. The Python prefix was cloned from the existing RaiSim 1 environment; this was not a fresh pip-wheel installation. The research `Environment.hpp`, `cfg.yaml` and robot models were unchanged.

- All 1,289 files in the 31 historical training runs matched their pre-move SHA-256 hashes; the runs contain 338 checkpoints. Old local paths remain compatibility links.
- The earlier workspace build rebuilt the native extension using the workspace's own Python, SDK and Eigen prefixes.
- `doctor` confirmed that the package, native extension and actual loaded `libraisim.so.1.1.8` came from that workspace.
- Replay of the preserved `2026-10-10-21-43-35/full_10000.pt` completed 22 simulated seconds / 2200 control steps with zero episode terminations in this rollout. Export also completed.
- Seven unittest cases passed, including launcher checks from a foreign working directory with inherited RaiSim 2 paths, rejection of a RaiSim 2 SDK and missing-configuration diagnostics. Documentation links, public Python syntax and active robot assets passed the release check.

The launcher changes process paths and command selection; it does not change the controller or training formulation. Local migration checks, environment inventories and build/replay/export logs stay in the Git-ignored workstation records described by [RESUME.md](../RESUME.md).

## Direct setup.py and task-file workflow — 2026-10-17

The original workflow was checked with the named Conda environment `hopper_nature`, using direct file execution:

- From the repository root, `python setup.py build_ext --inplace` rebuilt successfully without extra prefix arguments. CMake selected the local RaiSim 1.1.8/Eigen SDK and the active environment's Python 3.10.19.
- From `raisimGymTorch/env/envs/hopper/`, direct `python runner.py` execution completed a headless smoke run with 4 environments, 2 threads and 2 PPO updates. Its output was isolated under the workstation's validation folder, not mixed into the historical runs.
- Direct `python tester.py` execution replayed the preserved `full_10000.pt` bundle for 22 simulated seconds / 2200 steps with zero episode terminations in this rollout.
- Eight unittest cases passed. The new regression case executes both task files with site packages disabled and `PYTHONPATH` removed, from the task directory and a separate temporary directory. Documentation/source/robot checks also passed.

Only task-entry import paths and SDK discovery were changed. The research configuration, controller logic and robot assets were unchanged. The smoke run verifies command execution rather than policy performance; the Unity GUI was not started by these checks.

## Hopper naming and README installation check — 2026-10-17

The task directory and native extension are now `hopper`; training results are under `raisimGymTorch/data/hopper/`. All 1,289 files in the 31 historical runs retained their SHA-256 hashes, including 338 checkpoints. `Environment.hpp`, `HeightMap.hpp` and `cfg.yaml` are byte-identical to their pre-rename versions. The launcher folder was removed; export now runs directly through the task's `export.py`, and source/documentation checks run through unittest.

Two installations were checked:

- The existing `hopper_nature` environment rebuilt the renamed extension, completed two PPO updates with 4 environments and 2 threads, replayed the existing `2026-10-10-21-43-35/full_10000.pt` for 22 simulated seconds / 2200 steps with zero episode terminations, and exported its policy.
- A separate minimal Conda environment was created from cached Python packages, then installed the README's CPU PyTorch wheel and `requirements.txt`. It used Python **3.10.21**, PyTorch **2.5.1+cpu**, system GCC **11.4.0** and CMake **3.22.1**. `pip check` passed. A clean source copy containing no prior build, environment registrations or training output rebuilt successfully using `python setup.py build_ext --inplace`. The SDK/Eigen files were copied from the verified local RaiSim 1.1.8 dependency; the existing workstation activation key was used.
- In the clean copy, direct `python runner.py` completed two PPO updates with 4 environments and 2 threads, saving to the default `raisimGymTorch/data/hopper/<timestamp>/` path. Its untrained `full_0.pt` replayed for 2200 steps with 17 episode terminations and exported successfully. The same fresh environment also replayed the preserved trained checkpoint for 2200 steps with zero episode terminations in that rollout.
- Eight current unittest cases passed: CLI help/direct execution and checkpoint diagnostics, documentation links, entry point syntax and active robot assets/mass. Direct entry point checks disable site packages and remove `PYTHONPATH`.
- With inherited `LD_LIBRARY_PATH` and `PYTHONPATH` removed, the existing environment replayed the preserved checkpoint for another 2200 steps. The process's loaded-library map confirmed that every RaiSim library came from this project's `.local/runtime/raisimLib/`. The clean source build also resolved its RaiSim libraries from its own SDK copy without an external workspace path.

Local logs, the fresh environment's package versions and smoke-run artifacts are preserved under `.local/validation/` with `hopper-` and `readme-` prefixes. The temporary environment and source copy were removed after validation. These checks verify installation and execution, rather than policy performance.

## Project-local Unity and recording — 2026-10-18

Direct task entry points now accept `--viewer`. They launch only `.local/runtime/raisimLib/raisimUnity/linux/raisimUnity.x86_64` inside the checkout, wait for the native server's connection status, and close the owned client on exit. Manual two-terminal startup and `--headless` remain available. A small native connection-status accessor was added; the research environment, configuration and robot assets were unchanged.

- Rebuilt the native extension with `python setup.py build_ext --inplace` in `hopper_nature`.
- Ran `python runner.py --viewer --num-envs 4 --num-threads 2 --iterations 2 --no-tensorboard` with output isolated under the local validation directory and inherited `LD_LIBRARY_PATH` / `PYTHONPATH` removed. The project-local Unity connected and two PPO updates completed. With desktop/GPU access, the existing Conda build selected CUDA for this short run; this is not a full GPU-training benchmark.
- The client generated `2026-10-18-00-12-41policy_0.mp4` inside its project-local `Screenshot/` directory. FFprobe parsed it as H.264, 1024 × 800, 188 frames, 3.134 seconds. An extracted frame was visually inspected and showed the robot and connected Unity interface. This recording used the untrained iteration-0 policy.
- Ran `python tester.py --viewer --weight .../full_10000.pt --duration 22` with the preserved policy. The internal Unity connected, all 2200 control steps completed, and this rollout had zero episode terminations. The client closed after replay. The tester does not automatically record video.
- Repeated the two-update training and 22-second replay without a viewer using `--headless`.
- The Linux extension now links the complete selected RaiSim SDK with runtime search paths that take priority over inherited workspace library paths. Another 2200-step replay with the workstation's legacy `LD_LIBRARY_PATH` still set verified that all five loaded RaiSim libraries came from this checkout. A clean source export in a different directory was also built with system GCC 11.4 / CMake 3.22 and replayed for 2200 steps; all five RaiSim libraries came from that export’s own SDK despite the inherited legacy paths.
- Twelve unittest cases passed, including refusal to reuse an already connected client or a legacy viewer on `PATH`, local client launch without inherited library/Python paths, child cleanup, mutually exclusive viewer/headless options, and the existing CLI/source/documentation/robot checks.
- The local client executable and managed application assembly match the pinned public upstream revision. Its tracked `gui_settings.xml` already enables automatic connection to `127.0.0.1:8080`, so this workflow does not depend on workstation-specific Unity settings.

The workstation's `raisimUnity` alias was updated to this same internal executable. This alias is a convenience for the existing machine; new users can use `--viewer` directly after the README installation. Local logs, video metadata and an extracted frame are preserved under `.local/validation/unity-*`; automatically launched Unity logs are under `.local/logs/`.

## Remaining evidence

- A fresh OS installation has not been checked. The fresh Conda/pip-wheel check above reused the existing machine's system packages, verified SDK files and activation key.
- License acquisition and activation on another machine have not been checked.
- Full CUDA training, Windows/macOS and hardware execution have not been checked here. The local Unity/recording check above used the existing desktop, GPU driver and activation key.
- A representative final checkpoint, full training run, measured tracking metrics and a recorded simulation demo are still to be supplied. Maintainer-supplied hardware recordings are now included in [the demo record](demos.md); their association with a specific policy checkpoint is not yet documented.

When a result bundle is selected, add its source revision, model revision, configuration, checkpoint/CSV checksums, evaluation commands and measurements to this record. Record its demo intervals in [demos.md](demos.md).
