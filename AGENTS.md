# Hopper Nature repository context

Read [RESUME.md](RESUME.md) before changing this project. It records the last handoff, current robot/environment, saved artifacts and remaining work. If `.local/README.md` exists on this workstation, read it for local environment and checkpoint paths; `.local/` is intentionally excluded from Git.

- The project is parked after its public documentation/demo update. Preserve the current research configuration when doing maintenance.
- Active task: `raisimGymTorch/env/envs/hopper/`; native module `hopper`, training output `raisimGymTorch/data/hopper/`.
- Active robot: `rsc/Hop2_Parallel_Nature_20261008/`. Older model/calibration directories are historical evidence, not disposable build output.
- Use Python 3.10 and **RaiSim 1.1.8** with the existing requirements. RaiSim 2 instructions are not interchangeable.
- The maintainer prefers the original workflow: `conda activate hopper_nature`, `python setup.py build_ext --inplace` at the repository root, then `python runner.py` / `python tester.py` from `raisimGymTorch/env/envs/hopper/`. Preserve direct-file execution without requiring a wrapper, editable install or manually set `PYTHONPATH`.
- The build discovers `.local/runtime/raisimLib` by default and requires RaiSim 1.1.8 exactly. Explicit CMake prefixes remain supported. Keep machine-local dependencies outside Git and preserve the parked research configuration.
- Module execution (`python -m raisimGymTorch.env.envs.hopper.runner` / `tester` / `export`) also works. Use the task’s `export.py --weight ...` for portable checkpoint export. The maintainer requested removal of the launcher folder; keep build/train/replay commands direct.
- Preserve checkpoint bundles: `full_N.pt`, `meanN.csv`, `varN.csv`, `cfg.yaml`, and the associated `Environment.hpp`. `.local/` and data folders may contain irreplaceable research artifacts despite being ignored by Git. Avoid blanket cleanup such as `git clean -fdx`.
- Original camera clips and environment inventories stay local. Curated public MP4/GIF assets and their provenance live under `assets/demos/`.
- For documentation/tooling and entry point changes, run `python -m unittest discover -s tests -v`. This includes documentation links, Python syntax, active robot assets/mass and public CLI checks. Rebuild the native extension after C++ changes and use short headless simulation checks when behavior may change.
- Preserve the distinction between a successful software check and measured policy/hardware performance. YouTube URL, representative pretrained policy and recording-to-checkpoint correspondence are still pending; do not invent them.
