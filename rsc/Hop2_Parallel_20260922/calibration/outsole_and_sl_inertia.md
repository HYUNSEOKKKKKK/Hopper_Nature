# Outsole measurement scope and S/L inertia reconciliation

This revision updates only `Hop2_Parallel_20260820.urdf`. `hopper_cam.urdf` is
outside the calibration target.

## Confirmed Foot measurement scope

The user confirmed that the measured 837 g includes `foot_2_verHopping1`,
the eight bolts, and the attached tire/rubber outsole. The eight bolts weigh
48 g, so the remaining 789 g is **foot_2 plus outsole**, not the bare CAD part.

The mass accounting remains:

```
CAD whole Foot assembly             0.929141 kg
CAD measured-subset counterpart    -0.724284 kg
Measured subset including outsole  +0.837000 kg
Current Foot mass                  1.041857 kg
```

The 0.204857 kg of unmeasured parts retains CAD mass, as requested. Do not add
the outsole mass a second time. The 0.112716 kg residual also contains possible
CAD mass errors in the part and bolts; it is not a measured outsole mass.

Foot COM and inertia numerics are retained as a provisional prior estimate.
The old estimate distributes the residual over the selected CAD subset. Outsole
presence alone does not determine its separate mass, footprint or COM. In
addition, the old report-frame conversion needs reconciliation with the STEP
Foot registration. The STEP geometrical mapping is
`(X_URDF,Y_URDF,Z_URDF)=(-Y_CAD,X_CAD,Z_CAD-80 mm)`; the old numerical COM matches
`(-Y_report,-X_report,-Z_report)`. The report and STEP frames have not been
independently shown to be identical, so no unsupported frame change is applied.

The user elected to retain the existing two-box Foot collision and the 80 mm
ankle-to-flat-bottom distance. Its lower faces are consistent with an ideal
6.35 mm outsole allowance on both the flat and inclined CAD sole. The exact
physical pad shape and loaded deformation remain unmeasured.

## S/L composite inertia correction applied

The previous S/L visual edit verified 202/320 mm pin spacing but did not update
the rod inertials. The masses of Dice plus rod already agree with the rounded
CAD mass reports. However, their previous combined COM-inertia traces were
1.8801 times (S) and 2.0440 times (L) the provided CAD traces.

The CAD reports are transformed to the URDF reference frame using
`(X_URDF,Y_URDF,Z_URDF)=(Z_CAD,X_CAD,Y_CAD-Y_eye)`, with
`Y_eye=-96.5 mm` for S and `-155.5 mm` for L. Pin axes and asymmetric fork-bolt
geometry establish this mapping. SolidWorks positive-product off-diagonal
entries are negated before rotating the COM tensor into the URDF frame.

The existing Dice mass (0.037914 kg), COM (zero) and diagonal COM tensor
`[0.000002,0.000004,0.000004] kg*m^2` are retained as a **provisional partition**.
Rod COM and inertia are solved by subtracting that Dice contribution from the
whole-assembly spatial inertia using the parallel-axis theorem. This restores
the assembly COM and full COM tensor in the reference configuration, without
assigning the whole assembly to the rod or counting Dice twice.

| Rod | COM X (m) | COM Z (m) | Ixx (kg*m^2) | Iyy (kg*m^2) | Izz (kg*m^2) |
| --- | ---: | ---: | ---: | ---: | ---: |
| S | 0.000025644400 | 0.084408542972 | 0.000800961552250 | 0.000807444802804 | 0.000009875940554 |
| L | 0.000012225326 | 0.138158414615 | 0.002425260969736 | 0.002431743355101 | 0.000010525435365 |

The small off-diagonal terms are also applied; see the URDF and JSON for all
six entries. Masses are unchanged. Retained S/L total masses differ from the
rounded CAD reports by 4 mg and 1 mg, within the reports' 10 mg resolution.
Reported CAD COM has 0.01 mm resolution; output digits are numerical precision,
not claimed physical accuracy.

Exact lower-eye bushing ownership and individual Dice properties remain to be
confirmed. Small flexible-eye/bushing rotations in the CAD snapshot are
idealized in the reference frame. This is a composite-consistent improvement,
not a claim that the two moving bodies have individually measured inertials.

## Validation and reproducibility

`recalculate_sl_inertia.py` contains the supplied raw CAD values and generates
`sl_inertia_reconciliation.json`. It checks positive principal moments, the
principal-moment triangle inequalities, and composite COM/tensor reconstruction.
The exported XML is independently checked against that reconstruction. Only
the S/L rod COM and inertia numbers change. Foot numerical parameters, all
link masses, joint parameters, closure settings, and mesh references are held.
No dynamic simulator experiment has been run for this revision.

Current total link mass remains **12.958629 kg**, leaving **1.041371 kg**
unallocated relative to the user's 14 kg whole-robot measurement. This is not
automatically assigned to cables. Thigh's part boundary and inertial remain
separate pending audit items.
