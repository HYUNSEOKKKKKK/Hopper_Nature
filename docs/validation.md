# Validation record

Initial release check: **2026-10-16**. Workspace consolidation check: **2026-10-17**. These checks cover release tooling and local simulation execution. They do not report trained policy or hardware performance.

## Environment

| Item | Observed value |
| --- | --- |
| OS | Ubuntu 22.04, Linux x86_64, kernel 6.8 |
| Compiler / CMake | GCC 11.4.0 / CMake 3.22.1 |
| Python | 3.10.19 |
| PyTorch | Existing Conda CPU build `2.5.1.post303`; CUDA unavailable |
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
| Documentation/source checks | Relative file links, public Python syntax, active URDF mesh references and 17.100 kg model mass checked by `scripts/check_release.py` |

Replay used the smoke run's **untrained** `full_0.pt`. It had 14 episode terminations with automatic resets. Completion verifies execution, configuration loading and the command schedule, not successful continuous locomotion. Reward and timing from this short run are not presented as benchmark results.

## Reproduce the local check

After installation, run:

```bash
python scripts/check_release.py
python -m unittest discover -s tests -v

python -m raisimGymTorch.env.envs.hound.runner \
  --num-envs 4 --num-threads 2 --iterations 2 \
  --headless --no-tensorboard

# Substitute the actual timestamp printed by the runner:
python -m raisimGymTorch.env.envs.hound.tester \
  --weight raisimGymTorch/data/dhal_one_leg/<timestamp>/full_0.pt \
  --duration 22 --headless --no-realtime

python scripts/export_policy.py \
  --weight raisimGymTorch/data/dhal_one_leg/<timestamp>/full_0.pt
```

Replace `<timestamp>` before executing the commands. GitHub Actions runs only the checks that do not need a licensed simulator.

## Workstation handoff check — 2026-10-16

The current native extension was rebuilt during local cleanup, including the final environment settings committed in `029f4d5`. The existing local `2026-10-10-21-43-35/full_10000.pt` checkpoint was copied with its configuration, environment source and matching normalization files into the workstation's Git-excluded handoff folder. Its saved `Environment.hpp` matched the current source byte for byte.

A headless replay completed **22 simulated seconds / 2200 steps with zero episode terminations** for that one rollout. The documentation checks and four CLI unittest cases also passed. This check establishes that the preserved bundle can be loaded and run on the original workstation; it does not establish tracking accuracy, robustness or a recording-to-checkpoint association. The earlier smoke-run results above used a different, untrained checkpoint and remain a separate record. Local build/replay logs and dependency snapshots are indexed in [RESUME.md](../RESUME.md).

## One-folder workspace check — 2026-10-17

The project, its historical training runs, a dedicated Python 3.10 Conda prefix and the local RaiSim 1.1.8/Eigen/Unity dependencies were consolidated into one workstation directory. The Python prefix was cloned from the existing RaiSim 1 environment; this was not a fresh pip-wheel installation. The research `Environment.hpp`, `cfg.yaml` and robot models were unchanged.

- All 1,289 files in the 31 historical training runs matched their pre-move SHA-256 hashes; the runs contain 338 checkpoints. Old local paths remain compatibility links.
- `scripts/hopper.py build` rebuilt the native extension using the workspace's own Python, SDK and Eigen prefixes.
- `doctor` confirmed that the package, native extension and actual loaded `libraisim.so.1.1.8` came from that workspace.
- Replay of the preserved `2026-10-10-21-43-35/full_10000.pt` completed 22 simulated seconds / 2200 control steps with zero episode terminations in this rollout. Export also completed.
- Seven unittest cases passed, including launcher checks from a foreign working directory with inherited RaiSim 2 paths, rejection of a RaiSim 2 SDK and missing-configuration diagnostics. Documentation links, public Python syntax and active robot assets passed the release check.

The launcher changes process paths and command selection; it does not change the controller or training formulation. Local migration checks, environment inventories and build/replay/export logs stay in the Git-ignored workstation records described by [RESUME.md](../RESUME.md).

## Remaining evidence

- A clean OS / new Python environment installation using the documented pip wheel was not performed; simulation used the existing Conda PyTorch build listed above.
- License acquisition and activation on another machine have not been checked.
- CUDA training, the Unity client, viewer video recording, Windows/macOS and hardware execution have not been checked here.
- A representative final checkpoint, full training run, measured tracking metrics and a recorded simulation demo are still to be supplied. Maintainer-supplied hardware recordings are now included in [the demo record](demos.md); their association with a specific policy checkpoint is not yet documented.

When a result bundle is selected, add its source revision, model revision, configuration, checkpoint/CSV checksums, evaluation commands and measurements to this record. Record its demo intervals in [demos.md](demos.md).
