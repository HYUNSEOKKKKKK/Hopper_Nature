# Hopper Nature repository context

Read [RESUME.md](RESUME.md) before changing this project. It records the last handoff, current robot/environment, saved artifacts and remaining work. If `.local/README.md` exists on this workstation, read it for local environment and checkpoint paths; `.local/` is intentionally excluded from Git.

- The project is parked after its public documentation/demo update. Preserve the current research configuration when doing maintenance.
- Active task: `raisimGymTorch/env/envs/hound/`; Python module name `hound`, log task name `dhal_one_leg`.
- Active robot: `rsc/Hop2_Parallel_Nature_20261008/`. Older model/calibration directories are historical evidence, not disposable build output.
- Use Python 3.10 and **RaiSim 1.1.8** with the existing requirements. RaiSim 2 instructions are not interchangeable.
- Use `python -m raisimGymTorch.env.envs.hound.runner` / `tester` and `scripts/export_policy.py`. The historical task `export.py` retains a workstation-specific experiment reference.
- Preserve checkpoint bundles: `full_N.pt`, `meanN.csv`, `varN.csv`, `cfg.yaml`, and the associated `Environment.hpp`. `.local/` and data folders may contain irreplaceable research artifacts despite being ignored by Git. Avoid blanket cleanup such as `git clean -fdx`.
- Original camera clips and environment inventories stay local. Curated public MP4/GIF assets and their provenance live under `assets/demos/`.
- For documentation/tooling changes, run `python scripts/check_release.py`; use the public CLI unittest suite for entry point changes. Rebuild the native extension after C++ changes and use the short headless simulation checks when behavior may change.
- Preserve the distinction between a successful software check and measured policy/hardware performance. YouTube URL, representative pretrained policy and recording-to-checkpoint correspondence are still pending; do not invent them.
