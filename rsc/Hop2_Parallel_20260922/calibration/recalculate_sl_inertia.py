"""Reconcile S/L composite CAD inertia with the existing two-body URDF split.

This is a candidate calculation, not a direct URDF editor. The individual Dice
inertial is retained provisionally. Mesh shape identifies Dice with the lower
Eye_end region; exact bushing ownership still needs a selected-component report.
SolidWorks positive-product off-diagonals are negated for a mechanics tensor.
"""
from pathlib import Path
import json
import numpy as np

ROOT = Path(__file__).parent
R = np.array([[0., 0., 1.], [1., 0., 0.], [0., 1., 0.]])
DICE_MASS = .037914
DICE_COM = np.zeros(3)
DICE_I = np.diag([.000002, .000004, .000004])

def parallel_axis(c):
    return np.eye(3) * (c @ c) - np.outer(c, c)

def tensor_sw(a):
    xx, xy, xz, yy, yz, zz = a
    return np.array([[xx, -xy, -xz], [-xy, yy, -yz], [-xz, -yz, zz]]) * 1e-9

def inertia_fields(I):
    return dict(ixx=I[0, 0], ixy=I[0, 1], ixz=I[0, 2],
                iyy=I[1, 1], iyz=I[1, 2], izz=I[2, 2])

data = [
    dict(name="S", urdf_link="Left_Ankle_Link", cad_mass_kg=.17226,
         retained_rod_mass_kg=.134342, eye_cad_mm=[0., -96.5, 0.],
         cad_com_mm=[0., -30.67, .02],
         cad_positive_products_g_mm2=[1022118.30, -9.30, -.19,
                                      13875.96, -147.27, 1013635.03],
         old_com_m=[.000021, 0, .083422],
         old_inertia=np.diag([.001708, .001714, .000010])),
    dict(name="L", urdf_link="Right_Ankle_Link", cad_mass_kg=.20829,
         retained_rod_mass_kg=.170375, eye_cad_mm=[0., -155.5, 0.],
         cad_com_mm=[0., -42.49, .01],
         cad_positive_products_g_mm2=[3027705.35, .19, -.19,
                                      14525.44, -280.55, 3019222.96],
         old_com_m=[.000017, 0, .137377],
         old_inertia=np.diag([.005596, .005602, .000011])),
]

result = dict(
    status="provisional_Dice_partition_composite_CAD_reconciled",
    units="kg, m, kg*m^2",
    cad_to_urdf_rotation=R.tolist(),
    transform_evidence=[
        "STEP lower eye axis intersections: S (0,-96.5,0), L (0,-155.5,0) mm.",
        "STEP ball centers: S (0,105.5,0), L (0,164.5,0) mm.",
        "URDF V1 meshes extend along +Z from lower eye to the ball.",
        "STEP fork-end M3 bolt heads map to negative URDF X near -14 mm; both V1 meshes have the same asymmetric negative-X features.",
    ],
    caveats=[
        "Retains existing Dice mass, COM, and tensor; these are not newly measured CAD properties.",
        "Exact assignment of the lower-eye bushings between Dice and rod remains to be checked using selected-component CAD properties.",
        "Retains existing higher-precision URDF composite masses; these differ from rounded CAD reports by 4 mg (S) and 1 mg (L), both below the reports' 10 mg resolution.",
        "CAD COM input is rounded to 0.01 mm; numerical precision in outputs does not imply better physical accuracy.",
        "Supplied flexible-assembly CAD snapshot is reconciled to the straight-link URDF frame; tiny bushing/eye rotations visible in STEP are idealized.",
    ],
    retained_Dice=dict(mass_kg=DICE_MASS, com_m=DICE_COM.tolist(),
                       inertia_kg_m2=DICE_I.tolist()),
    links=[],
)
for d in data:
    m = d["retained_rod_mass_kg"]
    mt = m + DICE_MASS
    c = R @ (np.array(d["cad_com_mm"]) - np.array(d["eye_cad_mm"])) / 1000
    I = R @ tensor_sw(d["cad_positive_products_g_mm2"]) @ R.T
    c_rod = (mt * c - DICE_MASS * DICE_COM) / m
    I0 = I + mt * parallel_axis(c)
    I_rod = I0 - DICE_I - DICE_MASS * parallel_axis(DICE_COM) - m * parallel_axis(c_rod)
    rec_c = (m * c_rod + DICE_MASS * DICE_COM) / mt
    rec_I = I_rod + m * parallel_axis(c_rod - rec_c) + DICE_I + DICE_MASS * parallel_axis(DICE_COM - rec_c)
    eigen = np.linalg.eigvalsh(I_rod)
    assert np.min(eigen) > 0 and 2 * eigen[-1] <= eigen.sum()
    assert np.max(np.abs(rec_c-c)) < 1e-14 and np.max(np.abs(rec_I-I)) < 1e-14
    old_c = np.array(d["old_com_m"])
    old_comp_c = m * old_c / mt
    old_comp_I = d["old_inertia"] + m * parallel_axis(old_c-old_comp_c) + DICE_I + DICE_MASS * parallel_axis(DICE_COM-old_comp_c)
    out = dict(
        name=d["name"], urdf_link=d["urdf_link"],
        cad_report_mass_kg=d["cad_mass_kg"], retained_composite_mass_kg=mt,
        mass_difference_from_CAD_kg=mt-d["cad_mass_kg"],
        eye_cad_mm=d["eye_cad_mm"], composite_com_urdf_m=c.tolist(),
        composite_inertia_com_urdf_kg_m2=I.tolist(),
        rod_mass_kg=m, rod_com_m=c_rod.tolist(),
        rod_inertia_com_kg_m2=I_rod.tolist(),
        rod_inertia_urdf_fields=inertia_fields(I_rod),
        principal_inertias_kg_m2=eigen.tolist(),
        reconstruction_COM_max_error_m=float(np.max(np.abs(rec_c-c))),
        reconstruction_inertia_max_error_kg_m2=float(np.max(np.abs(rec_I-I))),
        old_composite_trace_ratio_to_CAD=float(np.trace(old_comp_I)/np.trace(I)),
    )
    result["links"].append(out)

(ROOT / "sl_inertia_reconciliation.json").write_text(json.dumps(result, indent=2) + "\n")
print(json.dumps(result, indent=2))
