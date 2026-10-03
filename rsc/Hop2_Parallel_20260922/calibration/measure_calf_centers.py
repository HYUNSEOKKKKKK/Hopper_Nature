"""Measure calf mesh end-axis separation, independently of nominal length.

Usage:
    python measure_calf_centers.py --output calf_mesh_centers.json

Source meshes are never modified. Coordinates are converted from their raw
meter convention to millimeters. Circle centers are inferred from vertices;
202/320 mm nominal lengths are not input to the fits.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from mesh_audit import BASE, NS, read, fit_circle_ransac


def fit_summary(points, radius_range):
    center, radius, inliers, residual = fit_circle_ransac(
        points, *radius_range, tol=0.04, iterations=10000
    )
    return {
        'center_projected_mm': center.tolist(),
        'radius_mm': float(radius),
        'unique_inlier_count': len(inliers),
        'radial_rms_mm': float(np.sqrt(np.mean(residual ** 2))),
        'radial_max_abs_mm': float(abs(residual).max()),
    }


def measure(path):
    root, vertices, faces, normals = read(path)
    v = vertices * 1000.0
    top_filter = (v[:, 2] > v[:, 2].max() - 23.0) & (abs(v[:, 1]) < 6.01)
    bottom_filter = (abs(v[:, 2]) < 6.0) & (abs(v[:, 1]) < 6.0) & (abs(v[:, 0]) > 7.9)
    bottom = fit_summary(v[bottom_filter][:, [1, 2]], (4.8, 5.2))
    top = fit_summary(v[top_filter][:, [0, 2]], (3.8, 4.2))
    top_z = top['center_projected_mm'][1]
    bottom_z = bottom['center_projected_mm'][1]

    # Independent least-squares circle fit to a plane through the upper bore.
    # Point selection uses the fitted z vicinity, not a nominal pin distance.
    plane = v[(abs(v[:, 1]) < 1e-4) & (abs(v[:, 2] - top_z) < 6.0)]
    projected = plane[:, [0, 2]]
    radius = np.linalg.norm(projected - [0.0, top_z], axis=1)
    projected = projected[abs(radius - 4.0) < 0.06]
    deterministic = None
    if len(projected) >= 3:
        a = np.column_stack([2 * projected, np.ones(len(projected))])
        b = (projected * projected).sum(axis=1)
        sol = np.linalg.lstsq(a, b, rcond=None)[0]
        fitted_radius = np.sqrt(sol[2] + sum(sol[:2] ** 2))
        residual = np.linalg.norm(projected - sol[:2], axis=1) - fitted_radius
        deterministic = {
            'plane': 'Y = 0 mm',
            'point_count': len(projected),
            'center_XZ_mm': sol[:2].tolist(),
            'radius_mm': float(fitted_radius),
            'radial_rms_mm': float(np.sqrt(np.mean(residual ** 2))),
        }

    unit = root.find('c:asset/c:unit', NS)
    transforms = []
    for node in root.findall('.//c:visual_scene//c:node', NS):
        for element in node:
            tag = element.tag.rsplit('}', 1)[-1]
            if tag in ('matrix', 'translate', 'rotate', 'scale', 'lookat', 'skew'):
                transforms.append({'type': tag, 'value': element.text})
    return {
        'file': path.name,
        'sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
        'vertices': len(v),
        'triangles': len(faces),
        'explicit_unit': None if unit is None else unit.attrib,
        'up_axis': root.findtext('c:asset/c:up_axis', namespaces=NS),
        'scene_transforms': transforms,
        'raw_coordinate_to_mm_scale': 1000.0,
        'bounds_mm': [v.min(axis=0).tolist(), v.max(axis=0).tolist()],
        'bottom_X_axis_fit_projected_YZ': bottom,
        'top_Y_axis_fit_projected_XZ': top,
        'perpendicular_pin_axis_separation_mm': abs(top_z - bottom_z),
        'upper_Y0_circle_independent_check': deterministic,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mesh-dir', type=Path, default=BASE)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    paths = sorted(args.mesh_dir.glob('URDF_Calf_Link_[SL]_*.dae'))
    if len(paths) != 4:
        raise SystemExit(f'Expected four S/L meshes; found {len(paths)} in {args.mesh_dir}')
    report = {
        'method': {
            'units': 'mm',
            'random_seed': 12345,
            'ransac_iterations': 10000,
            'radial_inlier_tolerance_mm': 0.04,
            'projected_vertex_deduplication_decimal_places': 8,
            'least_squares_refinement_iterations': 5,
            'bottom_filter': 'abs(Z)<6 and abs(Y)<6 and abs(X)>7.9',
            'bottom_axis': 'X',
            'bottom_circle_radius_range_mm': [4.8, 5.2],
            'top_filter': 'Z>max(mesh.Z)-23 and abs(Y)<6.01',
            'top_axis': 'Y',
            'top_circle_radius_range_mm': [3.8, 4.2],
            'distance': 'abs(top_axis.Z - bottom_axis.Z); axes X and Y are perpendicular',
            'interpretation': 'Mesh-derived cylinder-axis separation, not bounding-box length or certified physical tolerance.',
        },
        'meshes': [measure(path) for path in paths],
    }
    content = json.dumps(report, indent=2, ensure_ascii=False) + '\n'
    if args.output:
        args.output.write_text(content, encoding='utf-8')
    print(content, end='')


if __name__ == '__main__':
    main()
