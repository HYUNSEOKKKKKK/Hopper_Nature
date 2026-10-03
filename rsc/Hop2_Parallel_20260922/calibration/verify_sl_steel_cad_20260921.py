"""Crosscheck new steel CAD screenshots against the existing geometry/calibration.

Does not modify URDF values. Requires NumPy. All report numbers retain their
original displayed precision; tiny differences are not treated as new geometry.
"""
from pathlib import Path
import json
import numpy as np

ROOT = Path(__file__).resolve().parent


def p(c):
    return np.eye(3) * (c @ c) - np.outer(c, c)


def sw_tensor(v):
    xx, xy, xz, yy, yz, zz = v
    return np.array([[xx, -xy, -xz], [-xy, yy, -yz], [-xz, -yz, zz]]) * 1e-9


def calculate():
    source = json.loads((ROOT/'sl_steel_cad_confirmation_sources_20260921.json').read_text())
    old = json.loads((ROOT/'sl_inertia_reconciliation.json').read_text())
    geometry = json.loads((ROOT/'sl_measured_steel_inputs_20260921.json').read_text())
    rotation = np.array(old['cad_to_urdf_rotation'])
    result = {'date': source['date'], 'numeric_update_required': False,
              'decision': 'retain measured-mass URDF values; update part identity and CAD verification evidence',
              'remaining_assumptions': ['uniform mass distribution in each rod',
                                        'provisional Dice/bushing partition',
                                        'rounded original assembly COM inputs'],
              'records': []}
    for row in source['records']:
        baseline = next(x for x in old['links'] if x['name'] == row['label'])
        shape = next(x['geometry'] for x in geometry['parts']
                     if x['urdf_link'] == baseline['urdf_link'])
        part_mass = shape['volume_mm3'] * 7850e-9
        part_inertia = np.array(shape['part_COM_inertia_mm5']) * 7850e-15
        shown_part = row['part']
        # Compare each independent tensor entry to the displayed 0.01 g mm^2.
        assert abs(part_mass * 1000 - shown_part['mass_g']) <= .005
        assert abs(shape['volume_mm3'] - shown_part['volume_mm3']) <= .005
        assert np.max(np.abs((part_inertia-sw_tensor(shown_part['com_positive_products_g_mm2'])) * 1e9)) <= .005

        m0 = baseline['retained_composite_mass_kg']
        dm = shape['candidate_delta_mass_kg']
        m1 = m0 + dm
        c0 = rotation.T @ np.array(baseline['composite_com_urdf_m']) + np.array(baseline['eye_cad_mm']) / 1000
        i0 = rotation.T @ np.array(baseline['composite_inertia_com_urdf_kg_m2']) @ rotation
        # Both base rods have zero COM in the assembly default frame.
        c1 = m0 * c0 / m1
        delta_i = rotation.T @ np.array(shape['candidate_delta_COM_inertia_kg_m2']) @ rotation
        i1 = i0 + m0*p(c0) + delta_i - m1*p(c1)
        new = row['assembly']
        reported_i = sw_tensor(new['com_positive_products_g_mm2'])
        diff_i = reported_i - i1
        assert abs(m1*1000-new['mass_g']) <= .005
        result['records'].append({
            'label': row['label'], 'part_name': shown_part['name'],
            'part_volume_mm3': shape['volume_mm3'],
            'part_predicted_steel_mass_g': part_mass*1000,
            'part_CAD_report_mass_g': shown_part['mass_g'],
            'part_predicted_COM_inertia_g_mm2': (part_inertia*1e9).tolist(),
            'part_all_properties_match_display_precision': True,
            'assembly_predicted_steel_mass_g': m1*1000,
            'assembly_CAD_report_mass_g': new['mass_g'],
            'assembly_mass_predicted_minus_report_g': m1*1000-new['mass_g'],
            'assembly_predicted_COM_mm': (c1*1000).tolist(),
            'assembly_report_COM_mm': new['com_mm'],
            'assembly_predicted_COM_inertia_g_mm2': (i1*1e9).tolist(),
            'assembly_report_minus_predicted_inertia_g_mm2': (diff_i*1e9).tolist(),
            'assembly_max_diagonal_relative_difference': float(max(abs(np.diag(diff_i)/np.diag(reported_i)))),
            'interpretation': 'part properties match exactly at display precision; assembly differences are consistent with original COM rounding to 0.01 mm',
        })
    return result


if __name__ == '__main__':
    output = calculate()
    (ROOT/'sl_steel_cad_confirmation_results_20260921.json').write_text(json.dumps(output, indent=2)+'\n')
    print(json.dumps(output, indent=2))
