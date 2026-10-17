# Attribution and dependency notices

The repository retains its existing [MIT license](LICENSE), including the RaiSim Tech copyright notice. This documentation update does not replace upstream licenses or claim authorship of inherited implementations.

| Component | Provenance / terms |
| --- | --- |
| RaiSimGymTorch environment infrastructure | RaiSim Tech; retain source headers and the root MIT notice |
| Hopper / HOUND environment lineage | The Git history and configured upstream remote identify [GiJeongKim03/hound](https://github.com/GiJeongKim03/hound); retain original notices and history |
| Vendored pybind11 2.9.0 | [Bundled license](thirdParty/pybind11/LICENSE), BSD-style |
| RaiSim 1.1.8 runtime | Separately licensed binary dependency from [raisimTech/raisimLib](https://github.com/raisimTech/raisimLib); its engine terms and activation requirements apply independently |
| PyTorch, AdamP, NumPy, TensorBoard, ruamel.yaml, Eigen | External dependencies; their distributions retain their own licenses |
| Robot assets and calibration records | Stored in `rsc/`; preserve their embedded provenance and measurement assumptions |

The historical repository also contains Windows RaiSim DLLs and a debug symbol artifact. They are not required for the documented Linux build; their presence does not grant rights beyond their upstream terms. This update adds no new simulator binaries or activation material.

`setup.py` retains the inherited RaiSimGymTorch package-author attribution and identifies the repository source license as MIT, matching the existing root license. This does not relicense the proprietary simulator or third-party materials.

The README cover is generated from this repository's URDF/DAE assets by `assets/images/render_robot.py`. Recorded hardware clips were supplied by the project maintainer; their source and edits are recorded in [docs/demos.md](docs/demos.md). Project layout and presentation follow the maintainer's [PiPER Manipulation](https://github.com/HYUNSEOKKKKKK/Piper_manipulation) repository.
