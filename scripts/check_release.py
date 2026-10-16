"""Check public documentation, Python syntax and active robot asset references."""

import ast
from pathlib import Path
import re
from urllib.parse import unquote, urlsplit
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


def check_markdown_links():
    documents = [ROOT / name for name in ('README.md', 'CONTRIBUTING.md', 'NOTICE.md', 'RESUME.md', 'AGENTS.md')]
    documents += sorted((ROOT / 'docs').glob('*.md'))
    count = 0
    for document in documents:
        content = document.read_text()
        content = re.sub(r'```.*?```', '', content, flags=re.S)
        content = re.sub(r'`[^`]*`', '', content)
        content = re.sub(r'<!--.*?-->', '', content, flags=re.S)
        links = re.findall(r'\]\(([^\s)]+)(?:\s+[^)]*)?\)', content)
        links += re.findall(r'(?:src|href)="([^"]+)"', content)
        for link in links:
            parsed = urlsplit(link)
            if parsed.scheme or parsed.netloc or not parsed.path:
                continue
            target = (document.parent / unquote(parsed.path)).resolve()
            if not target.is_relative_to(ROOT) or not target.exists():
                raise ValueError(f'{document.relative_to(ROOT)}: missing local link {link}')
            count += 1
    print(f'Local documentation links: {count} resolved')


def check_python():
    files = sorted((ROOT / 'scripts').glob('*.py'))
    files += [ROOT / 'raisimGymTorch/helper/hopper_config.py']
    files += [ROOT / f'raisimGymTorch/env/envs/hound/{name}.py' for name in ('runner', 'tester')]
    for path in files:
        ast.parse(path.read_text(), filename=str(path))
    print(f'Public Python entry points: {len(files)} parsed')


def check_robot():
    header = (ROOT / 'raisimGymTorch/env/envs/hound/Environment.hpp').read_text()
    model = re.search(r'addArticulatedSystem\(resourceDir_\+"([^"]+)"\)', header)
    if model is None:
        raise ValueError('Cannot locate active robot model in Environment.hpp')
    path = ROOT / 'rsc' / model[1].lstrip('/')
    robot = ET.parse(path).getroot()
    for mesh in robot.findall('.//mesh'):
        if not (path.parent / mesh.get('filename')).is_file():
            raise ValueError(f'Missing active robot mesh: {mesh.get("filename")}')
    mass = sum(float(node.get('value')) for node in robot.findall('link/inertial/mass'))
    if abs(mass - 17.100) > 1e-6:
        raise ValueError(f'Active model mass {mass} disagrees with documented 17.100 kg')
    print(f'Active URDF: parsed, all mesh references exist, mass={mass:.6f} kg')


if __name__ == '__main__':
    check_markdown_links()
    check_python()
    check_robot()
