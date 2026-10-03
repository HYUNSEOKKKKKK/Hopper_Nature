"""Check transcribed tensors and bearing spacing; no robot-file modifications."""
from pathlib import Path
import json
import numpy as np

HERE = Path(__file__).resolve().parent
data = json.loads((HERE / 'thigh_partition_20260916.json').read_text())

def tensor(report):
    xx, xy, xz, yy, yz, zz = report['L_g_mm2']
    return np.array([[xx, -xy, -xz], [-xy, yy, -yz], [-xz, -yz, zz]]) * 1e-9

checks = {}
for name, report in data['reports'].items():
    I = tensor(report)
    eig = np.linalg.eigvalsh(I)
    assert np.all(eig > 0), name
    assert eig[0] + eig[1] >= eig[2] - 2e-11, name
    # All reports shown to only 0.01 g*mm^2 precision.
    error = np.max(np.abs(eig - np.sort(report['principal_g_mm2']) * 1e-9))
    assert error < 2e-11, (name, error)
    checks[name] = {'eigenvalues_kg_m2': eig.tolist(), 'principal_max_error_kg_m2': float(error)}

single, pair = data['reports']['yt_single'], data['reports']['yt_pair']
# trace(I_pair)-2trace(I_single) = m_single*d^2 for equal bearings.
m = single['mass_g'] * .001
spacing = np.sqrt((np.trace(tensor(pair)) - 2 * np.trace(tensor(single))) / m)
expected = data['old_step_reference']['yt_center_distance_mm'] * .001
assert abs(spacing - expected) < 1e-7
delta = np.array(pair['com_mm']) - data['old_step_reference']['yt_pair_center_mm_in_Thigh']
checks['yt_pair_geometry'] = {
    'center_distance_from_inertia_mm': float(spacing * 1000),
    'old_STEP_center_distance_mm': float(expected * 1000),
    'current_report_minus_old_STEP_COM_mm': delta.tolist(),
    'conclusion': 'Bearing spacing agrees; poses cannot be mixed without reconciliation.'
}
assert abs((296.74 + 22.32) * .001 - data['mass_subtotals_kg']['input_assembly_plus_six_bolts']) < 1e-12
assert abs((72.76 + 47.00) * .001 - data['mass_subtotals_kg']['long_link_plus_two_whole_bearings']) < 1e-12
out = HERE / 'thigh_partition_validation_20260916.json'
out.write_text(json.dumps(checks, indent=2) + '\n')
print(json.dumps({'tensor_reports_checked': len(data['reports']), **checks['yt_pair_geometry']}, indent=2))
