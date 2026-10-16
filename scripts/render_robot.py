"""Render the repository's URDF/DAE meshes for the README (NumPy + Pillow)."""

from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / 'rsc/Hop2_Parallel_Nature_20261008/Hop2_Parallel_Nature_20261008.urdf'


def transform(xyz=(0, 0, 0), rpy=(0, 0, 0)):
    x, y, z = rpy
    cx, cy, cz = np.cos(rpy)
    sx, sy, sz = np.sin(rpy)
    result = np.eye(4)
    result[:3, :3] = np.array([
        [cz*cy, cz*sy*sx-sz*cx, cz*sy*cx+sz*sx],
        [sz*cy, sz*sy*sx+cz*cx, sz*sy*cx-cz*sx],
        [-sy, cy*sx, cy*cx],
    ])
    result[:3, 3] = xyz
    return result


def origin(element):
    node = element.find('origin')
    if node is None:
        return np.eye(4)
    return transform(np.fromstring(node.get('xyz', '0 0 0'), sep=' '),
                     np.fromstring(node.get('rpy', '0 0 0'), sep=' '))


def mesh_triangles(path):
    root = ET.parse(path).getroot()
    ns = {'c': root.tag.split('}')[0][1:]}
    result = []
    # The shipped DAE assets have identity scene transforms and metre vertices.
    for node in root.findall('.//c:visual_scene//c:node', ns):
        if any(node.find('c:' + tag, ns) is not None for tag in ('matrix', 'translate', 'rotate', 'scale')):
            raise ValueError(f'Unexpected scene transform in {path}')
    for mesh in root.findall('.//c:mesh', ns):
        sources = {s.get('id'): np.fromstring(s.find('c:float_array', ns).text, sep=' ').reshape(-1, 3)
                   for s in mesh.findall('c:source', ns)}
        vertices = {v.get('id'): v.find('c:input', ns).get('source')[1:]
                    for v in mesh.findall('c:vertices', ns)}
        for triangles in mesh.findall('c:triangles', ns):
            inputs = triangles.findall('c:input', ns)
            stride = max(int(i.get('offset', '0')) for i in inputs) + 1
            vertex = next(i for i in inputs if i.get('semantic') == 'VERTEX')
            indices = np.fromstring(triangles.find('c:p', ns).text, sep=' ', dtype=int)
            indices = indices.reshape(-1, stride)[:, int(vertex.get('offset', '0'))].reshape(-1, 3)
            result.append(sources[vertices[vertex.get('source')[1:]]][indices])
    return np.concatenate(result)


def robot_triangles():
    robot = ET.parse(MODEL).getroot()
    joints = robot.findall('joint')
    children = {j.find('child').get('link') for j in joints}
    root = next(l.get('name') for l in robot.findall('link') if l.get('name') not in children)
    # Same nominal joint coordinates as Environment.hpp, in URDF joint order.
    angles = iter([0.4, -0.1, 0.0, 0.727127, -0.002475, 0.505687, 0.001563, 0.598060, 0.331160])
    poses = {root: np.eye(4)}
    for joint in joints:
        pose = origin(joint)
        if joint.get('type') != 'fixed':
            angle = next(angles)
            axis = np.fromstring(joint.find('axis').get('xyz'), sep=' ')
            axis /= np.linalg.norm(axis)
            skew = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
            rotation = np.eye(4)
            rotation[:3, :3] = np.eye(3) + np.sin(angle)*skew + (1-np.cos(angle))*(skew@skew)
            pose = pose @ rotation
        poses[joint.find('child').get('link')] = poses[joint.find('parent').get('link')] @ pose
    triangles = []
    box_faces = np.array([[0,1,3],[0,3,2],[4,6,7],[4,7,5],[0,4,5],[0,5,1],
                          [2,3,7],[2,7,6],[0,2,6],[0,6,4],[1,5,7],[1,7,3]])
    for link in robot.findall('link'):
        for visual in link.findall('visual'):
            mesh = visual.find('geometry/mesh')
            box = visual.find('geometry/box')
            if mesh is not None:
                tri = mesh_triangles(MODEL.parent / mesh.get('filename'))
                tri *= np.fromstring(mesh.get('scale', '1 1 1'), sep=' ')
            elif box is not None:
                size = np.fromstring(box.get('size'), sep=' ')
                vertices = np.array([[x, y, z] for x in [-.5,.5] for y in [-.5,.5] for z in [-.5,.5]]) * size
                tri = vertices[box_faces]
            else:
                continue
            pose = poses[link.get('name')] @ origin(visual)
            triangles.append(tri @ pose[:3, :3].T + pose[:3, 3])
    return np.concatenate(triangles)


def main():
    scale = 2
    canvas = Image.new('RGB', (1600*scale, 820*scale), '#0b1725')
    draw = ImageDraw.Draw(canvas)
    tri = robot_triangles()
    azimuth = np.deg2rad(38)
    right = np.array([np.cos(azimuth), np.sin(azimuth), 0])
    up = np.array([-.16*np.sin(azimuth), .16*np.cos(azimuth), .987])
    up /= np.linalg.norm(up)
    depth = np.cross(right, up)
    view = tri @ np.stack([right, up, depth], axis=1)
    lo, hi = view[:, :, :2].min(axis=(0,1)), view[:, :, :2].max(axis=(0,1))
    factor = min(560/(hi[0]-lo[0]), 650/(hi[1]-lo[1]))
    xy = (view[:, :, :2] - (lo+hi)/2) * factor
    xy[:, :, 0] += 1190
    xy[:, :, 1] = 405 - xy[:, :, 1]
    xy *= scale
    normals = np.cross(tri[:,1]-tri[:,0], tri[:,2]-tri[:,0])
    normals /= np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-12)
    light = np.array([-.4, -.5, .75])
    brightness = .35 + .65*np.abs(normals @ (light/np.linalg.norm(light)))
    colors = np.clip(brightness[:,None]*np.array([185,208,222]),0,255).astype(int)
    draw.ellipse((930*scale, 726*scale, 1460*scale, 756*scale), fill='#101f30')
    for i in np.argsort(view[:,:,2].mean(axis=1)):
        draw.polygon([tuple(p) for p in xy[i]], fill=tuple(colors[i]))
    font_dir = Path('/usr/share/fonts/truetype/dejavu')

    def text(x, y, value, size, color, bold=False):
        font = ImageFont.truetype(str(font_dir / ('DejaVuSans-Bold.ttf' if bold else 'DejaVuSans.ttf')), size*scale)
        draw.text((x*scale,y*scale),value,font=font,fill=color)

    text(80,76,'ROBOT LEARNING  /  SINGLE-LEG LOCOMOTION',19,'#68dcca',True)
    text(72,156,'HOPPER',100,'#f2f6fa',True)
    text(78,270,'NATURE',65,'#68dcca',True)
    draw.line((82*scale,390*scale,690*scale,390*scale),fill='#294252',width=2*scale)
    text(82,428,'A parallel ankle. A learned policy.',29,'#f2f6fa')
    text(82,480,'Simulation training with RaiSim and PPO.',23,'#a5b8ca')
    text(82,589,'3 ACTUATORS     100 Hz CONTROL     C++ / PYTORCH',17,'#68dcca',True)
    text(82,724,'Train locally  /  Replay checkpoints  /  Export policies',18,'#a5b8ca')
    text(1040,776,'URDF model render · 2026-10-08',16,'#a5b8ca')
    output = ROOT / 'assets/images/hopper-overview.png'
    output.parent.mkdir(parents=True, exist_ok=True)
    canvas.resize((1600,820), Image.Resampling.LANCZOS).save(output, optimize=True)
    print(output)


if __name__ == '__main__':
    main()
