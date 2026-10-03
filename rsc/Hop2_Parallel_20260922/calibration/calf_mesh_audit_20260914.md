# Calf S/L mesh length verification — 2026-09-14

The V1 meshes match the supplied SolidWorks STEP assembly's nominal pin-center
distances and the user's physical measurements. The Hopping1 meshes are about
4 mm shorter. Only the main Hop2_Parallel_20260820.urdf visual mesh references
were changed; hopper_cam.urdf was not edited.

| Source | Short S (mm) | Long L (mm) |
| --- | ---: | ---: |
| User's physical measurements | 202 | 320 |
| Hopper_verX2.STEP, Calf_link_S/L_verU1 | 202.000000 | 320.000000 |
| Hopping1 simplified DAE, fitted endpoint axes | 198.004639 | 316.000841 |
| V1 simplified DAE, fitted endpoint axes | 201.996969 | 319.998015 |

STEP measurement: traverse representation-to-representation assembly transforms;
measure from the intersection of Eye_end_20240529's radius-5 and radius-6 mm
cylindrical axes to the spherical center of E-GZW8_b_20240529. The STEP uses mm.
The two analytical eye axes intersect within 1e-7 mm. This does not use a part
bounding box, arbitrary cylinder surface origin, or a part-origin assumption.

STEP local centers (mm):
- S: eye approximately (0, -96.4999982, -0.0000017); ball approximately
  (0, 105.5000018, -0.0000017).
- L: eye approximately (0, -155.5000023, 0.0000050); ball approximately
  (0, 164.4999977, 0.0000050).

DAE measurement: fit circles to the lower X-axis pin (approximately radius 5 mm)
and upper Y-axis bore (approximately radius 4 mm), using mesh vertices converted
to mm. The bottom axis remains at the existing mesh origin. Both old and V1
files use the same raw coordinate frame, Y_UP metadata, identity scene node
transforms, and no explicit unit tag. The existing URDF uses zero visual origin
and scale 1 1 1. No extra Dice length is added: Dice and linkage joint origins
coincide. V1 fits differ from nominal by less than 0.005 mm, consistent with
the simplified mesh. Numerical fit precision is not manufacturing accuracy.

Applied changes:
- Left_Ankle_Link visual: URDF_Calf_Link_S_verV1_simplified.dae.
- Right_Ankle_Link visual: URDF_Calf_Link_L_verV1_simplified.dae.

Scope: endpoint geometry and compatibility with existing visual frames. This
does not prove every mesh feature is identical to the CAD, verify the native
SolidWorks assembly by rebuilding it, or validate simulator-specific up-axis
handling. The same conventions as the previous meshes are preserved.
Masses, COMs, inertias, drivetrain parameters, joint transforms, and the existing
202/320 mm closure configuration are unchanged by this edit. S/L links have no
collision element, so this is not a collision-model update. Earlier provisional
calibration assumptions remain provisional.

Reproduce mesh measurements (Python, numpy, scipy):
`python calibration/measure_calf_centers.py --mesh-dir .`

Reproduce the STEP measurements after placing the original Hopper_verX2.STEP
next to calibration/read_step_placements.py:
`python calibration/read_step_placements.py`
`python calibration/measure_step_centers.py`

The attached JSON files preserve fitted mesh values, source hashes, and STEP
center coordinates. The original supplied STEP and Pack and Go are not modified.
