"""Reproduce part-only measured-mass correction; does not edit the URDF.

Run with Python + NumPy. Reads the adjacent fixed input snapshot, so repeating
the calculation cannot apply the density correction twice.
"""
from pathlib import Path
import json
import numpy as np

ROOT = Path(__file__).resolve().parent

def P(c):
    return np.eye(3) * np.dot(c, c) - np.outer(c, c)

def tensor(f):
    return np.array([[f['ixx'], f['ixy'], f['ixz']],
                     [f['ixy'], f['iyy'], f['iyz']],
                     [f['ixz'], f['iyz'], f['izz']]])

def fields(I):
    return {k: float(I[i,j]) for k,i,j in
            [('ixx',0,0),('ixy',0,1),('ixz',0,2),
             ('iyy',1,1),('iyz',1,2),('izz',2,2)]}

def physical(I):
    e = np.linalg.eigvalsh(I)
    assert e[0] > 0
    assert e[-1] <= e[0] + e[1] + 1e-12
    return e.tolist()

def calculate():
    source = json.loads((ROOT/'sl_measured_steel_inputs_20260921.json').read_text())
    result = {'date': source['date'], 'units': source['units'],
              'status': 'measured_basepart_mass_with_confirmed_CAD_geometry_and_provisional_Dice_partition',
              'mass_source': 'user reports S=123 g, L=226 g; resolution unreported',
              'assumptions': [
                  'Measurements are single base rods, not whole S/L assemblies.',
                  'User corrected part identities to calf_link_S_1_verU1 / calf_link_L_1_verU1; new SolidWorks part properties validate the STEP mass distribution at displayed precision.',
                  'Existing installation transforms are retained; whole-assembly steel properties agree within propagated report rounding.',
                  'Existing basepart contribution is inferred at 2700 kg/m^3.',
                  'Measured mass is distributed uniformly over that CAD solid.',
                  'S45C grade is presumed, not established by weighing.',
                  'Unchanged Dice and bushing partition still provisional.',
                  'Aluminum and steel material stiffness are not modeled in rigid URDF.',
              ], 'links': []}
    delta_total = 0.
    for part in source['parts']:
        g = part['geometry']; old = part['before_link']
        m0 = old['mass_kg']; c0 = np.array(old['com_m'])
        I0 = tensor(old['inertia_fields'])
        mp0 = g['candidate_6061_mass_kg']; mp1 = part['measured_part_mass_kg']
        cp = np.array(g['urdf_link_frame_COM_m'])
        # mm^5 / mm^3 -> mm^2 -> m^2: inertia per kg of a uniform part.
        shape_I_per_kg = np.array(g['urdf_link_frame_COM_inertia_mm5']) / g['volume_mm3'] * 1e-6
        Ip0 = shape_I_per_kg * mp0; Ip1 = shape_I_per_kg * mp1
        dm = mp1-mp0; m1 = m0+dm; c1 = (m0*c0+dm*cp)/m1
        I1_origin = I0 + m0*P(c0) + (Ip1-Ip0) + dm*P(cp)
        I1 = I1_origin-m1*P(c1); I1 = .5*(I1+I1.T)
        # Independent representation as unchanged remaining hardware + new rod.
        mr = m0-mp0; cr = (m0*c0-mp0*cp)/mr
        Ir = I0+m0*P(c0)-Ip0-mp0*P(cp)-mr*P(cr)
        assert mr > 0
        physical(Ir); eigen = physical(I1)
        rebuilt = Ir+mr*P(cr-c1)+Ip1+mp1*P(cp-c1)
        error = float(np.max(np.abs(rebuilt-I1)))
        assert error < 1e-12
        r = {'urdf_link':part['urdf_link'], 'user_part_name':part['user_part_name'],
             'before_link':old, 'after_link':{'mass_kg':m1,'com_m':c1.tolist(),
                   'inertia_fields':fields(I1),'principal_inertias_kg_m2':eigen},
             'part_mass_before_kg':mp0, 'part_mass_measured_kg':mp1,
             'delta_mass_kg':dm, 'part_COM_urdf_m':cp.tolist(),
             'part_inertia_before_kg_m2':Ip0.tolist(),
             'part_inertia_after_kg_m2':Ip1.tolist(),
             'effective_density_kg_m3':mp1/(g['volume_mm3']*1e-9),
             'retained_hardware_mass_kg':mr,
             'retained_hardware_principal_inertias_kg_m2':physical(Ir),
             'reconstruction_max_error_kg_m2':error}
        result['links'].append(r); delta_total += dm
    result['total_before_kg'] = source['total_before_kg']
    result['delta_total_kg'] = delta_total
    result['total_after_kg'] = source['total_before_kg']+delta_total
    return result

if __name__ == '__main__':
    result = calculate()
    (ROOT/'sl_measured_steel_results_20260921.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
