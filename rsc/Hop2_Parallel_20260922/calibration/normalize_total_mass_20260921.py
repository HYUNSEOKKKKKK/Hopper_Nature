#!/usr/bin/env python3
"""Reproduce the 14 kg model from the immutable, pre-normalization baseline.

Python standard library only. Run from any directory. Does not rescale rotor
inertia or read the already-normalized URDF as its numerical input.
"""
from decimal import Decimal, getcontext, ROUND_HALF_EVEN
from pathlib import Path
import hashlib
import json
import re
import xml.etree.ElementTree as ET

getcontext().prec = 60
HERE = Path(__file__).resolve().parent
BASELINE = HERE / 'baseline_before_global_mass_20260921.urdf'
OUTPUT = HERE.parent / 'Hop2_Parallel_20260820.urdf'
BASELINE_SHA256 = '2cf7ba73442aa1278afbddf0bd54eb62ce5ea136ea00e153e1be864511f8c48e'
TARGET = Decimal('14.000000000000000000')
MASS_QUANTUM = Decimal('1e-18')
INERTIA_QUANTUM = Decimal('1e-24')
KEYS = ('ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz')


def fixed(value, quantum):
    return format(value.quantize(quantum, rounding=ROUND_HALF_EVEN), 'f')


def sha256(data):
    return hashlib.sha256(data).hexdigest()


def main():
    data = BASELINE.read_bytes()
    assert sha256(data) == BASELINE_SHA256, 'Baseline changed; create a new calibration revision.'
    text = data.decode('utf-8')
    tree = ET.fromstring(text)
    links = {e.attrib['name']: e for e in tree.findall('link') if e.find('inertial') is not None}
    assert len(links) == 11
    masses = {name: Decimal(e.find('inertial/mass').attrib['value']) for name, e in links.items()}
    total = sum(masses.values())
    factor = TARGET / total
    effective = {name: Decimal(fixed(m * factor, MASS_QUANTUM)) for name, m in masses.items()}
    # Make the serialized mass sum exactly 14 kg. At most a few 1e-18 kg:
    # this is decimal-output rounding, not a different physical allocation.
    balancing_link = max(masses, key=masses.get)
    rounding_correction = TARGET - sum(effective.values())
    effective[balancing_link] += rounding_correction
    rows = []
    updated = set()

    def update_link(match):
        block = match.group(0)
        node = ET.fromstring(block)
        name = node.attrib['name']
        if name not in links:
            return block
        source = links[name].find('inertial')
        inertia_before = source.find('inertia').attrib
        inertia_after = {k: fixed(Decimal(inertia_before[k]) * factor, INERTIA_QUANTUM) for k in KEYS}
        mass_after = fixed(effective[name], MASS_QUANTUM)
        count = 0

        def update_inertial(token):
            nonlocal count
            s = token.group(0)
            if s.startswith('<!--'):
                return s
            count += 1
            s, n = re.subn(r'<mass\b[^>]*/>', '<mass value="' + mass_after + '"/>', s)
            assert n == 1
            replacement = '<inertia ' + ' '.join(k + '="' + inertia_after[k] + '"' for k in KEYS) + '/>'
            s, n = re.subn(r'<inertia\b[^>]*/>', replacement, s)
            assert n == 1
            note = ('<!-- GLOBAL MASS NORMALIZATION 2026-09-21: baseline mass ' + str(masses[name])
                    + ' kg; effective mass ' + mass_after + ' kg.\n'
                    + '             All six COM-centered inertia entries scaled by the common factor. -->\n        ')
            return note + s

        block = re.sub(r'<!--.*?-->|<inertial\b[^>]*>.*?</inertial>', update_inertial, block, flags=re.S)
        assert count == 1
        updated.add(name)
        rows.append({
            'link': name,
            'baseline_mass_kg': str(masses[name]),
            'effective_mass_kg': mass_after,
            'allocated_residual_kg': str(effective[name] - masses[name]),
            'inertial_origin_unchanged': dict(source.find('origin').attrib),
            'baseline_com_inertia_kg_m2': dict(inertia_before),
            'effective_com_inertia_kg_m2': inertia_after,
        })
        return block

    text = re.sub(r'<link\b[^>]*>.*?</link>', update_link, text, flags=re.S)
    assert updated == set(links)
    old_header = re.search(r'    <!--\s+CALIBRATION WORKING COPY.*?-->', text, re.S)
    assert old_header is not None
    header = '''    <!--
      CURRENT CALIBRATED MODEL (2026-09-21): total link mass = 14.000 kg.
      User-requested global proportional distribution of unresolved mass:
      baseline total = 13.187198673293 kg; residual = 0.812801326707 kg;
      every link mass and all six COM-centered inertia entries are multiplied
      by 1.0616356321644795439547563032018071123483308513041.
      COM positions and inertial-frame orientations are unchanged.
      Physical-source comments below describe the PRE-NORMALIZATION baseline;
      effective link masses include the globally allocated residual, even for
      links whose baseline masses were measured. This is an assumed distribution.
      Joint rotor_inertia is unchanged; no separate D110 +46 g is added.
      Original measurements, baseline URDF, algorithm, and checks are retained
      in calibration/. See README_CURRENT_20260921.md for current model status.
      Remaining local provisional assumptions are not resolved by total-mass matching.
    -->'''
    text = text[:old_header.start()] + header + text[old_header.end():]
    text = text.replace('LEGACY numeric inertial block: Thigh calibration remains pending.',
                        'LEGACY baseline distribution: Thigh repartition remains pending.')
    text = text.replace('This note does not apply new numerical mass/inertia or rotor values.',
                        'This historical partition note does not supply new baseline values.')
    text = text.replace('Foot mass = 0.929141 - 0.724284 + 0.837 = 1.041857 kg.',
                        'Baseline Foot mass = 0.929141 - 0.724284 + 0.837 = 1.041857 kg.')
    text = text.replace('PROVISIONAL three-group physical calibration in URDF-aligned CS1:',
                        'PROVISIONAL BASELINE three-group calibration in URDF-aligned CS1:')
    text = text.replace('dumbbells stay at 3.000 kg; 0.063884 kg CAD hardware stays unchanged;',
                        'baseline dumbbells = 3.000 kg; baseline CAD hardware = 0.063884 kg;')
    result = ET.fromstring(text)
    assert sum(Decimal(e.attrib['value']) for e in result.findall('./link/inertial/mass')) == TARGET
    OUTPUT.write_text(text, encoding='utf-8')
    manifest = {
        'revision': '2026-09-21-global-mass-normalization',
        'target_source': 'User-reported whole-robot measurement, 14 kg; user explicitly requests proportional allocation to all links.',
        'target_total_mass_kg': str(TARGET),
        'baseline_total_mass_kg': str(total),
        'allocated_total_mass_kg': str(TARGET - total),
        'common_factor': str(factor),
        'increase_percent': str((factor - 1) * 100),
        'scope': 'All 11 link inertial masses and all six entries of each COM-centered inertia tensor, including measured links.',
        'distribution_assumption': 'Unresolved mass follows the existing distribution within each link and the existing mass proportions between links.',
        'com_xyz_and_rpy': 'Unchanged',
        'rotor_inertia': 'Unchanged; not part of this link-mass normalization.',
        'd110_reported_excess_kg': '0.046',
        'd110_excess_handling': 'Not added separately. Included in the unresolved total residual; per-unit scope and location remain unassigned.',
        'baseline_file': BASELINE.name,
        'baseline_sha256': BASELINE_SHA256,
        'normalized_file': OUTPUT.name,
        'normalized_sha256': sha256(OUTPUT.read_bytes()),
        'mass_decimal_places': 18,
        'inertia_decimal_places': 24,
        'rounding_balance_link': balancing_link,
        'rounding_balance_kg': str(rounding_correction),
        'link_values': rows,
        'future_update_rule': 'Revise the unnormalized physical baseline first, then recompute target/baseline sum. Never add identified mass on top of an already normalized model without rebudgeting the residual.',
    }
    (HERE / 'global_mass_normalization_20260921.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print(json.dumps({k: manifest[k] for k in ('baseline_total_mass_kg', 'target_total_mass_kg', 'common_factor', 'rounding_balance_kg', 'normalized_sha256')}, indent=2))


if __name__ == '__main__':
    main()
