# Contributing

Please open an issue with your Ubuntu, Python, PyTorch and RaiSim versions, exact command, configuration changes and error output. For behavior reports, include the source revision, robot revision, checkpoint iteration and matching normalization files. Do not attach activation keys.

For changes, keep the current `hound` extension name compatible, document any observation/action or checkpoint format changes, and rebuild after modifying C++ headers.

Run the checks that do not need a simulator:

```bash
python scripts/check_release.py
python -m unittest discover -s tests -v
```

For environment or training changes, also run the [two-update training check](docs/training.md#first-run) with your own licensed RaiSim installation, then replay and export its checkpoint. Report which checks were actually run; GPU, viewer and hardware behavior need their own validation.

Keep unprocessed camera videos, checkpoints, personal paths, license keys and build outputs outside commits. Curated, browser-compatible MP4 recordings and small GIF previews belong under `assets/demos/`, with source hashes and editing details in `docs/demos.md`. Retain existing copyright and third-party notices.
