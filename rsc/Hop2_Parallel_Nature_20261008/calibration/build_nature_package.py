"""Build the Nature variant from the user's 20260922 baseline and STEP geometry.

Run in the supplied working directory. Preserve measured source group masses;
reconcile the user-requested upper 2.550 kg and inferred lower 1.030 kg in a final stage.
The earlier 2.528 kg source stage below is historical; global normalization is disabled.
Intra-group mass distribution and cable placement are explicitly provisional.
"""
from pathlib import Path
from decimal import Decimal, getcontext
from zipfile import ZipFile, ZIP_DEFLATED
import csv
import hashlib
import json
import xml.etree.ElementTree as ET
import numpy as np

getcontext().prec = 50
WORK = Path('/workspace/scratch/813fa2b7d6a7')
SOURCE_ZIP = WORK / 'upload/Hop2_Parallel_20260922.zip'
GEOMETRY = WORK / 'nature_20261001/mesh_work/Body_Tethered_1_verNatureHop_geometry_properties.json'
SOURCE_DAE = WORK / 'nature_20261001/mesh_work/Body_Tethered_1_verNatureHop_body_local_CAD.dae'
NAME = 'Hop2_Parallel_Nature_20261001'
OUT = WORK / 'nature_20261001/deliverables' / NAME
CAL = OUT / 'calibration'
CAL.mkdir(parents=True, exist_ok=True)
R = np.array([[0., 0., 1.], [1., 0., 0.], [0., 1., 0.]])
IMU_MEASURED_KG = 0.020  # User approximate body-only measurement, supersedes 17.7 g nominal.
IMU_WITH_USB_KG = 0.062
IMU_USB_CABLE_KG = float(Decimal('0.062')-Decimal(str(IMU_MEASURED_KG)))
LOWER_FRAME_KG = 0.447
ETHERCAT_KG = 0.049
USB_LAN_HUB_KG = 0.034
USB_LAN_ETHERCAT_CABLE_KG = 0.050  # User approximate measurement.
IMU_SOURCE_URL = 'https://www.hbkworld.com/en/products/transducers/inertial-sensors/attitude-and-heading/3dm-gv7-ahrs'
PC_NOMINAL_KG = 1.4
UPPER_GROUP_KG = 2.528  # Includes killSwitch and its cables; not an extra group.
KILL_SWITCH_GROUP_KG = 0.133
CARRIER_BOARD_RECEIVER_KG = 0.806
BOARD_RECEIVER_KG = 0.119
CARRIER_KG = float(Decimal('0.806') - Decimal('0.119'))
UPPER_RESIDUAL_KG = float(Decimal('2.528') - Decimal('1.4') - Decimal('0.806') - Decimal('0.133'))
UPPER_RESIDUAL_NAME = 'upper_unlocalized_residual'
PC_SOURCE_URL = 'https://psref.lenovo.com/syspool/Sys/PDF/ThinkStation/ThinkStation_P360_Tiny/ThinkStation_P360_Tiny_Spec.html'
LOWER_NONBATTERY_MASS = sum(Decimal(str(m)) for m in [LOWER_FRAME_KG,ETHERCAT_KG,
    USB_LAN_HUB_KG,USB_LAN_ETHERCAT_CABLE_KG,IMU_MEASURED_KG,IMU_USB_CABLE_KG])
LOWER_GROUP_MASS = Decimal('3.100') + LOWER_NONBATTERY_MASS
MEASURED_BODY_MASS = LOWER_GROUP_MASS + Decimal(str(UPPER_GROUP_KG))
BODY_MASS = MEASURED_BODY_MASS
ROBOT_MASS = Decimal('10.42') + BODY_MASS
UPPER_SUPPORT_SPAN_M = {'x': 0.200, 'y': 0.200, 'z': 0.100}
UPPER_REACTIONS_G = {'x': {'plus': 1179.0, 'minus': 1329.0},
                     'y': {'plus': 1377.5, 'minus': 1149.0},
                     'z': {'plus': 1273.8, 'minus': 1257.5}}
# Z datum is provisional: each 5 mm carrier edge rests on a scale. Use the CAD
# edge-band centers, lower 110..115 -> 112.5 mm, upper 210..215 -> 212.5 mm.
# Their 100 mm center spacing matches the reported span. Actual centers of
# pressure within these contact bands are unmeasured; these are nominal datums.
UPPER_Z_LOWER_SUPPORT_M = 0.1125
UPPER_SUPPORT_MIDPOINT_M = {'x': 0.0, 'y': 0.0,
                          'z': UPPER_Z_LOWER_SUPPORT_M+UPPER_SUPPORT_SPAN_M['z']/2}

def write_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')

def vector(values):
    return ' '.join(f'{v:.12g}' for v in values)

def matrix_from_urdf(node):
    a = {k: float(v) for k, v in node.attrib.items()}
    return np.array([[a['ixx'],a['ixy'],a['ixz']], [a['ixy'],a['iyy'],a['iyz']], [a['ixz'],a['iyz'],a['izz']]])

def matrix_attributes(a):
    return {k: f'{a[i,j]:.15g}' for k,i,j in [('ixx',0,0),('ixy',0,1),('ixz',0,2),('iyy',1,1),('iyz',1,2),('izz',2,2)]}

with ZipFile(SOURCE_ZIP) as z:
    baseline = z.read('Hop2_Parallel_20260820/Hop2_Parallel_20260820.urdf')
    root = ET.fromstring(baseline, parser=ET.XMLParser(target=ET.TreeBuilder(insert_comments=True)))
    for name in sorted({e.get('filename') for e in root.findall('.//mesh')}):
        (OUT / name).write_bytes(z.read('Hop2_Parallel_20260820/' + name))
(CAL / 'source_Hop2_Parallel_20260922.xml').write_bytes(baseline)
root.set('xmlns:xacro', 'http://www.ros.org/wiki/xacro')

old_masses = {l.get('name'): Decimal(l.find('inertial/mass').get('value')) for l in root.findall('link')}
old_leg = sum(v for k,v in old_masses.items() if k != 'Temp_Weight')
leg_target = Decimal('10.42')
factor = leg_target / old_leg
leg_records = []

# Replace obsolete effective-mass comments; preserve physical source comments.
for parent in root.iter():
    for e in list(parent):
        if e.tag is ET.Comment and ('CURRENT CALIBRATED MODEL' in (e.text or '') or 'GLOBAL MASS NORMALIZATION 2026-09-21' in (e.text or '')):
            parent.remove(e)

for link in root.findall('link'):
    if link.get('name') == 'Temp_Weight':
        continue
    inertial = link.find('inertial')
    old_mass = old_masses[link.get('name')]
    new_mass = old_mass * factor
    inertial.find('mass').set('value', f'{new_mass:.18f}')
    for k,v in list(inertial.find('inertia').attrib.items()):
        inertial.find('inertia').set(k, f'{Decimal(v) * factor:.24f}')
    link.insert(0, ET.Comment(f' NATURE 2026-10-01: L_leg proportional correction {factor:.18f}; previous effective mass {old_mass} kg. COM unchanged, all six central tensor entries scaled with mass. '))
    leg_records.append({'link':link.get('name'), 'old_mass_kg':str(old_mass), 'new_mass_kg':str(new_mass), 'factor':str(factor)})

geometry = json.loads(GEOMETRY.read_text())
parts = {x['name']:x for x in geometry['components']}
known = {
    'battery_16S_1': 3.1,
    'body_1_tethered_verHopping1': LOWER_FRAME_KG,
    'EtherCAT_module_v6_GND': ETHERCAT_KG,
    'temp_USB_to_LAN_20240712': USB_LAN_HUB_KG,
    'killSwitch': KILL_SWITCH_GROUP_KG,
    'GV7_ASY_SingleBody': IMU_MEASURED_KG,
    'P360_20240712': PC_NOMINAL_KG,
    'NatureHop_pccarrier': CARRIER_KG,
    'nature_hopper_powerboard': BOARD_RECEIVER_KG,
}
upper_geometry_names = ['P360_20240712','NatureHop_pccarrier','nature_hopper_powerboard','button_big','killSwitch']
upper_volume = sum(parts[n]['volume_mm3'] for n in upper_geometry_names)
residual_weights = {n:parts[n]['volume_mm3']/upper_volume for n in upper_geometry_names}
masses = dict(known)
masses[UPPER_RESIDUAL_NAME] = UPPER_RESIDUAL_KG
centers = {n:np.array(p['uniform_geometry_centroid_body_CAD_mm'])*1e-3 for n,p in parts.items()}
unit_tensors = {n:np.array(p['uniform_geometry_inertia_per_unit_mass_m2']) for n,p in parts.items()}
# Separate the two measured lower cables from their device masses. No cable
# routes or connector coordinates were supplied. A uniform straight segment
# between device CAD centroids is an explicit distribution proxy, not measured
# routing or a claim that the physical cable follows that straight path.
cable_specs = {
    'GV7_USB_cable': {'mass_kg':IMU_USB_CABLE_KG,
                     'from':'GV7_ASY_SingleBody','to':'P360_20240712'},
    'USB_LAN_EtherCAT_cable': {'mass_kg':USB_LAN_ETHERCAT_CABLE_KG,
                            'from':'temp_USB_to_LAN_20240712','to':'EtherCAT_module_v6_GND'},
}
cable_records = {}
for n,spec in cable_specs.items():
    a=centers[spec['from']];b=centers[spec['to']];d=b-a
    masses[n]=spec['mass_kg'];centers[n]=(a+b)/2
    unit_tensors[n]=((d@d)*np.eye(3)-np.outer(d,d))/12
    cable_records[n] = {
        'assigned_mass_kg':masses[n],'mass_group':'lower, excluded from measured upper 2.528 kg',
        'source':spec['from'],'destination':spec['to'],
        'endpoint_proxy_body_CAD_m':[a.tolist(),b.tolist()],
        'endpoint_proxy_body_link_m':[(R@a).tolist(),(R@b).tolist()],
        'COM_body_link_m':(R@centers[n]).tolist(),
        'central_inertia_body_link_kgm2':(R@(masses[n]*unit_tensors[n])@R.T).tolist(),
        'distribution':'uniform slender straight segment between device CAD centroid proxies',
        'unit_tensor_formula':'(||b-a||^2*identity - outer(b-a,b-a))/12',
        'status':'PROVISIONAL: connector locations, routing, coiled slack and connector mass distribution unmeasured. Segment length is a proxy and is not actual cable length.',
    }
# Residual is an explicit mass budget, not a measured button/component mass.
# Its unknown spatial distribution is provisionally spread over the upper CAD
# solids by volume. Measured carrier and board masses are kept separate.
residual_center = sum(residual_weights[n]*centers[n] for n in upper_geometry_names)
residual_unit_tensor = np.zeros((3,3))
for n in upper_geometry_names:
    d = centers[n]-residual_center
    residual_unit_tensor += residual_weights[n]*(unit_tensors[n]+(d@d)*np.eye(3)-np.outer(d,d))
centers[UPPER_RESIDUAL_NAME] = residual_center
unit_tensors[UPPER_RESIDUAL_NAME] = residual_unit_tensor
total = sum(masses.values())
center_cad = sum(masses[n]*centers[n] for n in masses) / total
I_cad = np.zeros((3,3))
component_records = []
for n,m in masses.items():
    Ii = m*unit_tensors[n]
    d = centers[n]-center_cad
    I_cad += Ii + m*((d@d)*np.eye(3)-np.outer(d,d))
    if n == UPPER_RESIDUAL_NAME:
        status = 'UNLOCALIZED RESIDUAL: 2.528 measured upper INCLUDING switch/cables - 1.400 nominal PC - 0.806 measured carrier/board/receiver - 0.133 switch/cables subtotal = 0.189 kg. Includes button/cables/other unaccounted items and possible PC nominal-mass error; assumed spread over upper five CAD geometries by volume. Not assigned entirely to button; not a separately measured physical component'
    elif n == 'NatureHop_pccarrier':
        status = 'DERIVED MEASURED CARRIER: 0.806 kg carrier+board+receiver minus 0.119 kg board+receiver = 0.687 kg; user identifies PLA printed part. COM/inertia use uniform effective density in CAD geometry; infill/wall distribution unmeasured'
    elif n == 'nature_hopper_powerboard':
        status = 'MEASURED GROUP: powerboard plus unmodeled receiver = 0.119 kg; receiver lumped into powerboard geometry and location per explicit user instruction'
    elif n == 'P360_20240712':
        status = 'PROVISIONAL MANUFACTURER MASS: P360 Tiny 1.4 kg maximum configuration, approximate and configuration-dependent; Tiny form factor inferred from CAD size, exact machine type not confirmed. COM/inertia remain uniform-CAD estimates, not manufacturer specifications'
    elif n == 'body_1_tethered_verHopping1':
        status = 'USER MEASURED 0.447 kg, supersedes previous 0.397 kg residual inferred from old 0.506/0.109 kg groups; same CAD part geometry'
    elif n == 'EtherCAT_module_v6_GND':
        status = 'USER MEASURED 0.049 kg, supersedes previous 0.047 kg; connecting cable is separate'
    elif n == 'temp_USB_to_LAN_20240712':
        status = 'USER MEASURED HUB ONLY 0.034 kg; 0.050 kg EtherCAT connecting cable represented separately; supersedes old derived hub-plus-cable 0.062 kg'
    elif n == 'killSwitch':
        status = 'MEASURED SUBGROUP: switch plus cables 0.133 kg included within measured upper 2.528 kg, counted once; represented by switch geometry'
    elif n == 'GV7_ASY_SingleBody':
        status = 'USER APPROXIMATE BODY-ONLY MEASUREMENT 0.020 kg; supersedes manufacturer nominal 0.0177 kg. Body-plus-USB cable measured 0.062 kg; remaining approximately 0.042 kg cable is a separate lower-group distribution, excluded from upper 2.528 kg'
    elif n in cable_specs:
        status = ('USER DERIVED APPROXIMATE MASS 0.062 minus approximately 0.020 = approximately 0.042 kg; ' if n=='GV7_USB_cable' else 'USER APPROXIMATE MEASURED CABLE MASS 0.050 kg; ') + cable_records[n]['status'] + ' Assigned to lower group, never added to upper 2.528 kg.'
    else:
        status = 'MEASURED component mass; uniform CAD geometry used for COM and inertia'
    component_records.append({'CAD_name':n if n in parts else None, 'record_name':n,
        'record_type':'CAD_component' if n in parts else ('uniform_segment_proxy' if n in cable_specs else 'unlocalized_mass_budget'),
        'assigned_mass_kg':m, 'status':status,
        'uniform_geometry_COM_body_CAD_m':centers[n].tolist(),
        'uniform_geometry_central_inertia_kgm2':Ii.tolist()})
body_com = R@center_cad
body_I = R@I_cad@R.T
cad_only_body_com = body_com.copy()
cad_only_body_I = body_I.copy()

# Upper XYZ measurement constrains the aggregate first moment, not individual
# component positions or second moments. Keep CAD component records as priors;
# retain the upper central tensor as an estimate, then recombine the
# calibrated upper aggregate with the unchanged lower component distributions.
def combine_distributions(records):
    mass = sum(v[0] for v in records)
    com = sum(m*c for m,c,I in records)/mass
    tensor = np.zeros((3,3))
    for m,c,I in records:
        d = c-com
        tensor += I+m*((d@d)*np.eye(3)-np.outer(d,d))
    return mass, com, tensor

upper_names = ['P360_20240712','NatureHop_pccarrier','nature_hopper_powerboard',
               'killSwitch',UPPER_RESIDUAL_NAME]
lower_names = [n for n in masses if n not in upper_names]
def prior_distribution(n):
    return masses[n], R@centers[n], R@(masses[n]*unit_tensors[n])@R.T
upper_mass, upper_cad_com, upper_I = combine_distributions(
    [prior_distribution(n) for n in upper_names])
upper_calibrated_com = upper_cad_com.copy()
for i,axis in enumerate(('x','y','z')):
    r = UPPER_REACTIONS_G[axis]
    upper_calibrated_com[i] = UPPER_SUPPORT_MIDPOINT_M[axis]+UPPER_SUPPORT_SPAN_M[axis]/2*(r['plus']-r['minus'])/(r['plus']+r['minus'])
lower_mass, lower_com, lower_I = combine_distributions(
    [prior_distribution(n) for n in lower_names])
_, body_com, body_I = combine_distributions([
    (upper_mass,upper_calibrated_com,upper_I), (lower_mass,lower_com,lower_I)])

upper_com_calibration = {
    'status':'approximate three-axis two-scale upper COM calibration; support geometry and absolute Z datum are nominal',
    'date':'2026-10-02', 'measured_upper_mass_kg':UPPER_GROUP_KG,
    'mass_source':'independent larger-scale measurement; reaction sums do not replace 2.528 kg',
    'frame':'Body_Nature URDF axes [X,Y,Z] = Body CAD [Z,X,Y]',
    'support_midpoints_link_m':UPPER_SUPPORT_MIDPOINT_M,
    'support_midpoint_status':'XY assumed symmetric about centered carrier CAD axes. Z midpoint=0.1625 m from CAD lower and upper contact-band centers 0.1125/0.2125 m. Actual centers of pressure not separately measured.',
    'contact_geometry_status':'User reports XY spacing approximately 200 mm and Z spacing 100 mm, used as effective reaction-line spacing. Actual finite support contact positions are not independently verified.',
    'z_support_datum':{'lower_support_link_z_m':UPPER_Z_LOWER_SUPPORT_M,
                       'lower_support_status':'assumed at center of 5 mm lower carrier edge band, not at outer mounting face; actual center of pressure unmeasured',
                       'carrier_CAD_bottom_top_link_z_m':[0.110,0.215],
                       'lower_contact_band_link_z_m':[0.110,0.115],
                       'upper_contact_band_link_z_m':[0.210,0.215],
                       'carrier_CAD_envelope_height_m':0.105,
                       'effective_upper_support_z_under_assumption_m':UPPER_Z_LOWER_SUPPORT_M+UPPER_SUPPORT_SPAN_M['z'],
                       'datum_caveat':'Reported 100 mm span matches CAD contact-band center spacing, while full outer height is 105 mm. Equal-center contact is an assumption. Under exact 100 mm span, shifting both support lines within the two bands permits +/-2.5 mm change in upper absolute COM; this is geometric sensitivity, not a statistical confidence interval.',
                       'source_image':'2a35ec12-4714-4ff9-9511-841da139a072.png'},
    'user_confirmed_second_test_rotated_90_degrees':True,
    'superseded_lateral_span_m':0.185,
    'direction':'killSwitch-side reaction is +Y in lateral test and -X in fore-aft test; Z test left 1273.8 g=upper/+Z, right 1257.5 g=lower/-Z per user',
    'formula':'q = support_midpoint + D/2 * (reaction_plus - reaction_minus)/(reaction_plus + reaction_minus)',
    'tests':{axis:{'nominal_support_span_m':UPPER_SUPPORT_SPAN_M[axis],
                   'reaction_plus_g':r['plus'],'reaction_minus_g':r['minus'],
                   'reaction_sum_g':r['plus']+r['minus'],
                   'reaction_sum_minus_reference_g':r['plus']+r['minus']-UPPER_GROUP_KG*1000,
                   'COM_offset_from_support_midpoint_m':float(upper_calibrated_com[i]-UPPER_SUPPORT_MIDPOINT_M[axis]),
                   'COM_distance_from_minus_support_m':float(upper_calibrated_com[i]-UPPER_SUPPORT_MIDPOINT_M[axis]+UPPER_SUPPORT_SPAN_M[axis]/2),
                   'dCOM_dSpan':(r['plus']-r['minus'])/(2*(r['plus']+r['minus']))}
             for i,(axis,r) in enumerate(UPPER_REACTIONS_G.items())},
    'upper_prior_COM_link_m':upper_cad_com.tolist(),
    'upper_calibrated_COM_link_m':upper_calibrated_com.tolist(),
    'upper_COM_change_link_m':(upper_calibrated_com-upper_cad_com).tolist(),
    'upper_central_inertia_link_kgm2':upper_I.tolist(),
    'inertia_status':'retained CAD-based upper aggregate central tensor; not identified by two-scale tests',
    'height_status':'Z relative to lower support measured from reaction ratio; absolute Body-frame Z uses provisional lower contact-band center datum 0.1125 m',
    'lower_mass_kg':lower_mass,'lower_COM_link_m':lower_com.tolist(),
    'lower_central_inertia_link_kgm2':lower_I.tolist(),
    'Body_prior_COM_link_m':cad_only_body_com.tolist(),
    'Body_prior_central_inertia_link_kgm2':cad_only_body_I.tolist(),
    'Body_COM_change_link_m':(body_com-cad_only_body_com).tolist(),
    'modeling_rule':'replace upper aggregate first moment only; retain its central second moment as an approximation; recombine upper and lower by parallel-axis theorem',
    'component_records_status':'body_components describe pre-upper-COM-calibration CAD and cable-proxy distributions; use this aggregate calibration for final Body properties',
    'limitations':['support midpoint and actual pad contact lines are approximate',
                   'fore-aft reaction sum is 20 g below the independent upper mass',
                   'absolute upper Z depends on assumed contact-band center datums; finite 5 mm edge width leaves a few mm geometric uncertainty',
                   'all central second moments remain CAD-based estimates; not measured by these tests',
                   'lower device COMs and tensors remain CAD-based estimates; two cable distributions use unmeasured straight-segment proxies'],
}

# The delivered mesh is rotated into Body_Nature axes, parallel to Thigh axes.
# ThighCAD -> BodyCAD translation [0,-147.7,0] mm, together with the verified
# CAD -> ThighURDF transform (R,[0,0,147.7]mm), gives zero mounting translation.
ns = {'c':'http://www.collada.org/2005/11/COLLADASchema'}
ET.register_namespace('', ns['c'])
dae_root = ET.parse(SOURCE_DAE).getroot()
arr = dae_root.find('.//c:float_array',ns)
native_vertices = np.fromstring(arr.text, sep=' ').reshape(-1,3)
vertices = native_vertices@R.T
arr.text = ' '.join(f'{v:.10g}' for v in vertices.ravel())
dae_name = 'Body_Tethered_1_verNatureHop.dae'
ET.ElementTree(dae_root).write(OUT / dae_name, encoding='utf-8', xml_declaration=True)

old_joint = root.find("joint[@name='09_temp_weight_joint']")
old_body = root.find("link[@name='Temp_Weight']")
joint_index = list(root).index(old_joint)
root.remove(old_joint)
root.remove(old_body)
joint = ET.Element('joint', {'name':'09_body_nature_joint','type':'fixed'})
ET.SubElement(joint,'origin',{'xyz':'0 0 0','rpy':'0 0 0'})
ET.SubElement(joint,'parent',{'link':'Thigh'})
ET.SubElement(joint,'child',{'link':'Body_Nature'})
root.insert(joint_index,joint)
body = ET.Element('link',{'name':'Body_Nature'})
body.append(ET.Comment(f' Nature body mass = {BODY_MASS} kg from upper 2.528 kg plus lower {LOWER_GROUP_MASS} kg. Lower consists of battery 3.100, frame 0.447, EtherCAT 0.049, USB-LAN hub 0.034, hub-EtherCAT cable approx 0.050, GV7 body approx 0.020 and its USB cable approx 0.042 kg. Both lower cables EXCLUDE upper measured mass. Old 0.506/0.109 kg group constraints and nominal GV7 0.0177 kg are superseded. Cable mass uses uniform segments between CAD device centroid proxies, not measured routes. Upper XYZ COM is retained: {vector(upper_calibrated_com)} m. XY assumes symmetric 200 mm supports; Z assumes 100 mm span with contact-band centers at 112.5/212.5 mm. Actual contact centers unverified. Upper central inertia remains the previous CAD estimate. Whole Body COM and tensor recomputed; see calibration/. '))
visual=ET.SubElement(body,'visual')
ET.SubElement(visual,'origin',{'xyz':'0 0 0','rpy':'0 0 0'})
vg=ET.SubElement(visual,'geometry')
ET.SubElement(vg,'mesh',{'filename':dae_name,'scale':'1 1 1'})
ET.SubElement(visual,'material',{'name':'grey'})

# Conservative height bands: exact envelopes of CAD component bounding boxes
# intersecting each band. These are collision approximations, not inertial solids.
bands=[(-0.01016,0.11),(0.11,0.215),(0.215,0.2336)]
boxes=[]
for low,high in bands:
    slices=[]
    for p in parts.values():
        b=np.array(p['bounds_body_CAD_mm'])*1e-3
        if min(b[1,1],high)-max(b[0,1],low) > 1e-9:
            b[0,1]=max(b[0,1],low); b[1,1]=min(b[1,1],high)
            slices.append(b)
    b=np.array([np.min([s[0] for s in slices],axis=0),np.max([s[1] for s in slices],axis=0)])
    c=R@b.mean(axis=0); size=R@(b[1]-b[0])
    collision=ET.SubElement(body,'collision')
    ET.SubElement(collision,'origin',{'xyz':vector(c),'rpy':'0 0 0'})
    cg=ET.SubElement(collision,'geometry')
    ET.SubElement(cg,'box',{'size':vector(size)})
    boxes.append({'center_body_URDF_m':c.tolist(),'size_body_URDF_m':size.tolist()})
inertial=ET.SubElement(body,'inertial')
ET.SubElement(inertial,'origin',{'xyz':vector(body_com),'rpy':'0 0 0'})
ET.SubElement(inertial,'mass',{'value':f'{BODY_MASS:.18f}'})
ET.SubElement(inertial,'inertia',matrix_attributes(body_I))
root.insert(joint_index+1,body)

header=f'''
  NATURE MODEL 2026-10-01 / 2026-10-02 lower measured parts and cables: L_leg = 10.420 kg; new Body = {BODY_MASS} kg;
  total = {ROBOT_MASS} kg, measured component/group sums (some approximate),
  not a whole-robot weighing.
  Scope assumption: L_leg excludes the complete upper Nature Body.
  Lower excluding battery = {LOWER_NONBATTERY_MASS} kg: frame 0.447,
  EtherCAT 0.049, USB-LAN hub 0.034, hub-to-EtherCAT cable approx 0.050,
  GV7 body approx 0.020 + its USB cable approx 0.042 kg (together 0.062 kg).
  Add lower battery 3.100 kg -> lower subtotal {LOWER_GROUP_MASS} kg.
  These replace old lower 0.506/0.109 kg group constraints and GV7 nominal 17.7 g.
  GV7 body and USB cable are lower-group masses, EXCLUDED from upper 2.528 kg,
  even though the USB cable connects to the upper PC. Neither cable is counted twice.
  Two lower cables are distributed uniformly between their endpoint device CAD
  centroids as provisional straight-line proxies; actual ports/routes/slack unknown.
  Upper 2.528 kg INCLUDES killSwitch and its cables.
  The 0.133 kg switch/cable mass is a subtotal, not an additional mass.
  PC uses provisional Lenovo P360 Tiny nominal 1.400 kg (maximum configuration,
  configuration-dependent; exact machine type unconfirmed). The measured upper
  assembly is measured at 2.528 kg; carrier = 0.687 kg from 0.806 minus 0.119 kg;
  powerboard INCLUDING receiver = 0.119 kg, lumped at powerboard per user.
  Unlocalized upper residual = 0.189 kg (button/cables/other mass and possible
  PC nominal error), spread over upper CAD geometries by volume as an estimate.
  The 0.806 kg subgroup and 0.133 kg switch subtotal are within the 2.528 kg budget.
  Lenovo COM/inertia specifications were not found; uniform CAD estimates remain.
  Upper XY COM uses two-scale tests with user-corrected nominal 200 mm spacing
  on BOTH axes and support midpoints assumed at centered carrier CAD axes.
  Z uses upper/lower reactions 1273.8/1257.5 g and reported 100 mm span,
  with supports ASSUMED at CAD edge-band centers Z=112.5 and 212.5 mm.
  The full outer height is 105 mm; actual pressure centers within the 5 mm
  contact edges remain unverified, leaving a few mm of absolute-Z uncertainty.
  Upper calibrated COM [m] = {vector(upper_calibrated_com)}; absolute Z is provisional.
  Upper central inertia is retained as a CAD estimate, not measured by scales.
  Measured upper mass/COM and its central tensor estimate are unchanged by this
  lower-only update. Whole Body properties are recombined by the parallel-axis theorem.
  Contact-line/midpoint uncertainty and reaction mass discrepancies are recorded.
  Previous 20260922 regular model: total 14 kg, old Temp_Weight 3.588328436715940859 kg.
  Old Body removed. Existing ten leg-link masses/tensors scaled by
  1.000799913507194218; their COMs, visuals, collisions, moving joints, limits and
  rotor_inertia values are unchanged. Existing physical partition limitations remain.
  Body visual vertices and inertial tensor are rotated from CAD [X,Y,Z]
  into link [Z,X,Y]. The new Body fixed joint is at Thigh origin, not the legacy
  dummy-weight +0.1 m offset. See README_Nature_20261001.txt and calibration/.
'''
root.insert(0,ET.Comment(header))
ET.indent(root,space='    ')
urdf_path=OUT/(NAME+'.urdf')
ET.ElementTree(root).write(urdf_path,encoding='utf-8',xml_declaration=True)

assumptions=[
    'L_leg=10.42 kg is the union of the ten existing non-Body URDF links, excluding upper Body and its cabling.',
    'Lower 3.742 kg and upper 2.528 kg are disjoint measured-component/group sums, not separately measured assembled totals. Upper includes killSwitch/cables; lower includes battery, GV7 body and its entire USB cable even though it connects to the upper PC.',
    'New lower component measurements replace old 0.506/0.109 kg bundled constraints. Frame 0.447, EtherCAT 0.049, hub 0.034, hub-EtherCAT cable approx 0.050, GV7+USB total 0.062, battery 3.100 kg. Old subtotals are historical only and are not added or enforced.',
    'GV7 body uses user approximate 0.020 kg, superseding manufacturer nominal 0.0177 kg. Its USB cable is the approximate residual 0.062-0.020=0.042 kg, represented separately. GV7 group total is included once.',
    'P360_20240712 uses provisional Lenovo P360 Tiny 1.4 kg maximum-configuration specification. Tiny is inferred from the compact CAD envelope, not confirmed by a machine type label. Exact configuration and actual unit mass remain unknown.',
    'The measured 0.806 kg carrier/board/receiver subgroup is included within the 2.528 kg upper assembly, not added to it.',
    'Carrier mass is 0.806 minus 0.119 = 0.687 kg; powerboard plus receiver is 0.119 kg. The receiver is explicitly lumped into powerboard CAD geometry/location per user instruction.',
    'The PLA carrier uses uniform effective density within its CAD geometry. Measured mass fixes its mass scale but does not identify actual printing infill/wall COM or inertia.',
    'After 1.400 kg nominal PC, 0.806 kg measured carrier/board/receiver subgroup, and 0.133 kg switch/cable subgroup, 0.189 kg remains within upper 2.528 kg. This is an unlocalized budget including button/cables/other items and possible PC specification error, not a measured cable or button mass. Its distribution is approximated by five upper CAD solid volume weights; measured component masses remain separate.',
    'Each CAD device central tensor uses uniform density within its CAD geometry, scaled to its assigned mass; the two unmodeled lower cables use separate uniform-segment tensors.',
    'Hub-EtherCAT cable 0.050 kg and GV7-PC USB cable 0.042 kg are uniform straight-segment distribution proxies between corresponding CAD device centroids. Ports, actual routes, bundled slack, and connector mass distribution are unmeasured; proxy segment lengths are not actual cable lengths. KillSwitch/cables remains represented within upper group as before.',
    'Upper XY COM uses reaction ratios and corrected approximate 200 mm spacing on both axes. Effective support lines are assumed symmetric about carrier CAD center; pad contact locations are not independently verified.',
    'Upper mass stays 2.528 kg despite reaction sums XY 2.5265/2.508 kg and Z 2.5313 kg; scale ratios identify approximate COM only. Central inertia remains a CAD estimate; Body inertia is recombined using the new aggregate COM.',
    'Z test uses upper 1273.8 g, lower 1257.5 g and user-reported 100 mm support spacing. CAD lower/upper 5 mm contact bands are Z=110..115 and 210..215 mm. Their centers 112.5/212.5 mm define the nominal supports. Actual centers of pressure are unmeasured; absolute Z remains provisional.',
    'body_components retains uncalibrated CAD device priors and lower cable-proxy distributions. Final Body properties require the upper_com_calibration aggregate first-moment correction; no physical CAD component or mesh was relocated.',
    'The 8.3284 g leg correction is proportional across ten links. It does not identify the actual cable mass locations.',
    'Body collision boxes conservatively fill some unoccupied interior space; the visual mesh is not used as collision geometry.',
]
model={
    'model':NAME, 'date':'2026-10-02', 'revision':'Apply lower measured device masses and two separate cable proxies; lower 3.742 kg, upper calibrated 2.528 kg unchanged, Body 6.270 kg, robot 16.690 kg.',
    'baseline_urdf_sha256':hashlib.sha256(baseline).hexdigest(),
    'mass_kg':{'baseline_robot':str(sum(old_masses.values())), 'baseline_old_Body':str(old_masses['Temp_Weight']),
        'baseline_L_leg':str(old_leg),'measured_L_leg':'10.42','L_leg_correction':str(leg_target-old_leg),
        'L_leg_factor':str(factor),'current_Body_component_group_sum':str(MEASURED_BODY_MASS),
        'lower_excluding_battery':str(LOWER_NONBATTERY_MASS),'lower_including_battery':str(LOWER_GROUP_MASS),
        'new_Body_group_sum':str(BODY_MASS),'new_robot_sum':str(ROBOT_MASS),
        'prior_revision_Body':'6.1517','prior_revision_robot':'16.5717','revision_mass_change':'0.1183'},
    'measured_Body_groups_kg':{'battery_16S_1':3.1,'body_1_tethered_verHopping1':LOWER_FRAME_KG,
        'EtherCAT_module_v6_GND':ETHERCAT_KG,'temp_USB_to_LAN_20240712':USB_LAN_HUB_KG,
        'USB_LAN_EtherCAT_cable_approx':USB_LAN_ETHERCAT_CABLE_KG,
        'GV7_body_and_USB_group':IMU_WITH_USB_KG,
        'GV7_body_approx_subtotal_NOT_ADDITIONAL':IMU_MEASURED_KG,
        'GV7_USB_approx_subtotal_NOT_ADDITIONAL':IMU_USB_CABLE_KG,
        'upper_assembly_INCLUDING_killSwitch_and_all_upper_cables':UPPER_GROUP_KG,
        'killSwitch_and_cables_subtotal_NOT_ADDITIONAL':KILL_SWITCH_GROUP_KG,
        'carrier_powerboard_receiver_subtotal_NOT_ADDITIONAL':CARRIER_BOARD_RECEIVER_KG,
        'powerboard_receiver_subsubtotal_NOT_ADDITIONAL':BOARD_RECEIVER_KG},
    'IMU_source':{'manufacturer':'MicroStrain by HBK','family':'3DM-GV7',
        'previous_manufacturer_nominal_mass_kg':0.0177,'assigned_body_mass_kg':IMU_MEASURED_KG,
        'mass_status':'user approximate measured body mass supersedes previous manufacturer nominal',
        'body_plus_USB_measured_kg':IMU_WITH_USB_KG,'derived_USB_cable_approx_kg':IMU_USB_CABLE_KG,
        'source_url':IMU_SOURCE_URL,'verified_date':'2026-10-01',
        'official_dimensions_mm':[36.2,36.6,10.2],
        'mounting':'below body_1_tethered_verHopping1; CAD placement retained'},
    'user_confirmed_scope':{'GV7_is_in_upper_PC_group':False,'battery_in_lower_body':True,'GV7_in_lower_body':True,
        'GV7_USB_is_in_upper_PC_group':False,'GV7_USB_in_lower_mass_group':True,
        'USB_LAN_hub_and_EtherCAT_cable_separate_masses':True,
        'upper_2p528_includes_killSwitch_and_cables':True,
        'upper_2p528_weighed_on_single_larger_scale':True},
    'superseded_lower_ledger':{'status':'historical only; do not add or enforce together with current measurements',
        'old_frame_board_hub_cables_group_kg':0.506,'old_board_hub_cables_group_kg':0.109,
        'old_frame_derived_kg':0.397,'old_EtherCAT_kg':0.047,'old_hub_plus_cables_derived_kg':0.062,
        'old_GV7_nominal_kg':0.0177},
    'lower_cable_proxies':cable_records,
    'PC_source':{'manufacturer':'Lenovo','inferred_model':'ThinkStation P360 Tiny',
        'model_identity_status':'inferred from user P360 name and compact CAD size; exact machine type unconfirmed',
        'source_url':PC_SOURCE_URL,'verified_date':'2026-10-01',
        'nominal_mass_kg':PC_NOMINAL_KG,'specification_qualifier':'1.4 kg maximum configuration; approximate, configuration-dependent',
        'official_dimensions_WDH_mm':[179,182.9,37],
        'CAD_envelope_body_axes_mm':(np.array(parts['P360_20240712']['bounds_body_CAD_mm'])[1]-np.array(parts['P360_20240712']['bounds_body_CAD_mm'])[0]).tolist(),
        'CAD_dimension_note':'186.06 x 43.5 x 187.1 mm envelope; compatible with compact Tiny form but not an exact dimensional match',
        'manufacturer_COM_inertia_found':False,
        'previous_volume_allocated_PC_mass_kg':1.465528598053115,
        'measured_upper_assembly_including_switch_kg':UPPER_GROUP_KG,
        'non_PC_mass_within_upper_kg':UPPER_GROUP_KG-PC_NOMINAL_KG},
    'upper_mass_budget':{'measured_total_kg':UPPER_GROUP_KG,'PC_nominal_kg':PC_NOMINAL_KG,
        'carrier_derived_measured_kg':CARRIER_KG,'powerboard_including_receiver_measured_kg':BOARD_RECEIVER_KG,
        'killSwitch_and_cables_included_subtotal_kg':KILL_SWITCH_GROUP_KG,
        'unlocalized_residual_kg':UPPER_RESIDUAL_KG,
        'residual_geometry_volume_weights':residual_weights,
        'residual_COM_body_CAD_m':residual_center.tolist(),
        'residual_unit_central_tensor_body_CAD_m2':residual_unit_tensor.tolist(),
        'button_has_no_separate_measured_mass':True,
        'receiver_geometry':'omitted; mass included once in powerboard at user request'},
    'assumptions':assumptions,'leg_links':leg_records,'body_components':component_records,
    'upper_com_calibration':upper_com_calibration,
    'Body_COM_link_m':body_com.tolist(),'Body_central_inertia_link_kgm2':body_I.tolist(),
    'Body_principal_moments_kgm2':np.linalg.eigvalsh(body_I).tolist(),
    'rotation_body_CAD_to_body_URDF':R.tolist(),'body_joint_xyz_m':[0,0,0],'body_joint_rpy_rad':[0,0,0],
    'collision_boxes':boxes,
    'cad_mounting_check':{'ThighCAD_to_BodyCAD_mm':[0,-147.7,0], 'ThighCAD_to_CalfCAD_mm':[0,-400,0],
        'CAD_to_ThighURDF_translation_mm':[0,0,147.7], 'mapped_Body_origin_mm':[0,0,0],
        'mapped_Calf_origin_mm':[0,0,-252.3], 'existing_knee_origin_mm':[0,0,-252.3]},
}
write_json(CAL/'nature_mass_model.json',model)
write_json(CAL/'upper_com_calibration.json',upper_com_calibration)
write_json(CAL/'lower_cable_proxies.json',cable_records)
(CAL/'source_body_geometry_properties.json').write_bytes(GEOMETRY.read_bytes())
with (CAL/'body_mass_ledger.csv').open('w',newline='',encoding='utf-8-sig') as f:
    w=csv.writer(f);w.writerow(['component_or_mass_budget','assigned_mass_kg','evidence_or_assumption'])
    for c in component_records:w.writerow([c['record_name'],f"{c['assigned_mass_kg']:.12f}",c['status']])

# Verify the concrete risks of this change: mass budget, valid inertias,
# unchanged leg geometry and actuator settings, new frame transform and assets.
check=ET.parse(urdf_path).getroot()
new_total=sum(Decimal(l.find('inertial/mass').get('value')) for l in check.findall('link'))
assert abs(new_total-ROBOT_MASS)<Decimal('1e-15')
assert abs(sum(masses.values())-float(BODY_MASS))<1e-12
assert 'GV7_ASY_SingleBody' not in upper_geometry_names
assert abs(sum(masses[n] for n in ['P360_20240712','NatureHop_pccarrier','nature_hopper_powerboard','killSwitch',UPPER_RESIDUAL_NAME])-UPPER_GROUP_KG)<1e-12
assert abs(masses['NatureHop_pccarrier']+masses['nature_hopper_powerboard']-CARRIER_BOARD_RECEIVER_KG)<1e-12
assert abs(masses['NatureHop_pccarrier']-0.687)<1e-12
assert abs(masses['nature_hopper_powerboard']-0.119)<1e-12
assert 'button_big' not in masses and abs(masses[UPPER_RESIDUAL_NAME]-0.189)<1e-12
assert abs(sum(residual_weights.values())-1)<1e-12
assert abs(upper_mass-UPPER_GROUP_KG)<1e-12
assert np.allclose(upper_calibrated_com[:2],[-0.005980861244019139,0.009044132198693845],atol=1e-14,rtol=0)
assert abs(upper_calibrated_com[2]-0.1628219689487615)<1e-14
assert np.allclose(body_com, [0.008117714111344,0.001372782375783,0.106992610435698],atol=1e-14,rtol=0)
assert abs(lower_mass-3.742)<1e-12
assert abs(masses['GV7_ASY_SingleBody']+masses['GV7_USB_cable']-IMU_WITH_USB_KG)<1e-12
assert all(n in lower_names for n in cable_specs)
for n in cable_specs:
    assert np.allclose(unit_tensors[n],unit_tensors[n].T,atol=1e-15)
    assert np.linalg.eigvalsh(unit_tensors[n]).min()>-1e-15
# Hierarchical recombination without COM calibration must reproduce the prior.
_, prior_com_check, prior_I_check = combine_distributions([
    (upper_mass,upper_cad_com,upper_I),(lower_mass,lower_com,lower_I)])
assert np.allclose(prior_com_check,cad_only_body_com,atol=1e-14,rtol=0)
assert np.allclose(prior_I_check,cad_only_body_I,atol=1e-14,rtol=0)
assert np.allclose(R@R.T,np.eye(3)) and np.linalg.det(R)>0
assert np.isfinite(vertices).all()
idx=np.fromstring(dae_root.find('.//c:triangles/c:p',ns).text,sep=' ',dtype=int)
assert idx.min()>=0 and idx.max()<len(vertices)
assert len(parts)==10
covered=np.zeros(len(vertices),dtype=bool)
for b in boxes:
    c=np.array(b['center_body_URDF_m']); half=np.array(b['size_body_URDF_m'])/2
    covered|=np.all(np.abs(vertices-c)<=half+1e-9,axis=1)
assert covered.all(),int((~covered).sum())
inertia_checks={}
for l in check.findall('link'):
    a=matrix_from_urdf(l.find('inertial/inertia')); eig=np.linalg.eigvalsh(a)
    assert np.isfinite(a).all() and eig.min()>0
    assert eig[-1] <= eig[0]+eig[1]+1e-10
    inertia_checks[l.get('name')]=eig.tolist()
for m in check.findall('.//mesh'):
    assert (OUT/m.get('filename')).is_file(),m.get('filename')
original=ET.fromstring(baseline)
for j in original.findall('joint'):
    if j.get('name')!='09_temp_weight_joint':
        nj=check.find(f"joint[@name='{j.get('name')}']")
        for child in j:
            assert nj.find(child.tag).attrib==child.attrib,(j.get('name'),child.tag)
for l in original.findall('link'):
    if l.get('name')=='Temp_Weight':continue
    nl=check.find(f"link[@name='{l.get('name')}']")
    assert nl.find('inertial/origin').attrib==l.find('inertial/origin').attrib
    for tag in ('visual','collision'):
        old_elements=l.findall(tag);new_elements=nl.findall(tag)
        assert len(old_elements)==len(new_elements)
        for a,b in zip(old_elements,new_elements):
            assert [(e.tag,e.attrib) for e in a.iter()]==[(e.tag,e.attrib) for e in b.iter()]
validation={'status':'passed','total_mass_kg':str(new_total), 'link_count':len(check.findall('link')),
    'joint_count':len(check.findall('joint')), 'body_mesh_vertices':len(vertices), 'body_mesh_triangles':len(idx)//3,
    'body_mesh_bounds_link_m':[vertices.min(axis=0).tolist(),vertices.max(axis=0).tolist()],
    'all_body_mesh_vertices_inside_collision_boxes':bool(covered.all()),'inertia_eigenvalues_kgm2':inertia_checks,
    'checks':['XML parse','mass sum','ten Body CAD geometries','upper measured 2.528 kg includes switch/cables exactly once','measured carrier and board subgroup mass closure','explicit upper residual without button double count','finite mesh coordinates and valid indices',
              'positive-definite inertias and principal triangle inequalities','mesh paths present',
              'leg COM/visual/collision unchanged','moving joint parameters and rotor inertias unchanged',
              'two-scale XY signs and corrected 200 mm spans; Z upper/lower ratio with 100 mm span','hierarchical aggregation reproduces previous CAD model before calibration',
              'upper mass/central inertia retained; final Body recombined by parallel-axis theorem',
              'new lower 3.742 kg mass closure; GV7 62 g counted once as body plus cable',
              'lower cable segments use positive-semidefinite central tensors; no upper-group membership'],
    'upper_COM_calibration_status':'approximate: symmetric XY supports each 200 mm; Z span 100 mm and supports at CAD edge-band centers Z=112.5/212.5 mm assumed; actual contact lines and datum not independently verified',
    'not_tested':['RaiSim/other simulator loading and dynamics','independent whole Body COM and central inertia measurement','support contact-line metrology and absolute Z support datum verification','actual lower cable routing/ports/slack distribution','exact PC machine type and individual mass','remaining measurement-group scope assumptions']}
write_json(CAL/'validation.json',validation)

readme=f'''Hop2 Parallel Nature / filename 2026-10-01 / updated 2026-10-02

실행할 파일: {NAME}.urdf
URDF와 같은 폴더의 DAE들을 함께 사용하세요. 상대 경로 기준입니다.
별도 전체 ZIP에는 이 URDF, 필요한 모든 DAE, 질량표 및 계산 근거가 있습니다.

질량 비교
기존 일반 20260922 URDF: 전체 14.000 kg
  기존 Body(Temp_Weight): 3.588328436715941 kg
  Body 제외 L_leg: 10.411671563284059 kg
이번 실측 L_leg: 10.420 kg (전선 포함, 상부 새 Body 제외로 해석)
  차이: +8.328436716 g, 보정계수: 1.000799913507194
새 Body 실측 부품/묶음 합(IMU와 케이블 포함): {MEASURED_BODY_MASS} kg
일부 수치는 사용자 근사 실측값이며, 완성 Body/로봇 전체를 한 번에 잰 값은 아닙니다.
새 Body 모델 질량: {BODY_MASS} kg
새 모델 전체: 약 {ROBOT_MASS} kg (부품/묶음 합계)

새 Body 질량표
  battery_16S_1: 3.100 kg
  body_1_tethered_verHopping1: 0.447 kg (사용자 표기 body_1_tethered_verHopping)
  EtherCAT_module_v6_GND: 0.049 kg
  temp_USB_to_LAN_20240712: 0.034 kg (허브 본체만)
  허브와 EtherCAT 연결선: 약 0.050 kg, 별도 질량 분포
  GV7_ASY_SingleBody 본체: 약 0.020 kg
  GV7 USB 케이블: 약 0.042 kg (=본체+케이블 0.062 - 본체 약 0.020)
    GV7 본체와 케이블 전체는 아래파트이며 윗파트 2.528 kg에 포함되지 않음
  배터리 제외 하부 합: {LOWER_NONBATTERY_MASS} kg
  배터리 포함 하부 합: {LOWER_GROUP_MASS} kg
  윗파트 전체: 2.528 kg (큰 저울 하나로 측정)
    P360 + NatureHop_pccarrier + powerboard + button + killSwitch 및 해당 케이블 포함
    killSwitch + 관련 cables: 0.133 kg, 위 2.528 kg 안에 이미 포함됨
    위 묶음에 포함된 carrier + powerboard + receiver 실측 소계: 0.806 kg
    그중 powerboard + receiver: 0.119 kg
    따라서 PLA carrier 단독: 0.806 - 0.119 = 0.687 kg
    미배정 잔여: 2.528 - PC 참고값 1.400 - 0.806 - killSwitch/cables 0.133 = 0.189 kg
합: 3.100 + 0.642 + 2.528 = {BODY_MASS} kg

이번 하부 질량 교체
직전 모델 Body 6.1517 kg -> 6.270 kg, 로봇 전체 16.5717 kg -> 약 16.690 kg.
증가분 118.3 g: 프레임 +50 g, EtherCAT +2 g,
허브/연결선 합 62 g -> 34+50=84 g으로 +22 g,
GV7 명목 17.7 g -> 실측 본체+USB 62 g으로 +44.3 g.
기존 하부 묶음 506 g 및 네트워크 묶음 109 g은 이번 세부 실측으로 대체했습니다.
이를 새 합계에 더하거나 강제로 유지하지 않습니다. 이전 IMU 명목 17.7 g도 대체했습니다.
윗파트 2.528 kg과 그 COM/중심 관성은 이 하부 보정에서 그대로 유지합니다.

이전 윗파트 질량 보정 이력 (이번 하부 변경보다 앞선 이력)
사용자는 윗파트에 killSwitch와 관련 케이블이 항상 포함된다고 확인했습니다.
이전 모델에서 윗파트 2.460 kg와 killSwitch 0.133 kg을 따로 더한 것은 중복이었습니다.
앞선 질량 보정에서 그 합 2.593 kg 대신 실제 윗파트 전체 2.528 kg을 적용했습니다.
그때 Body/전체 질량이 0.065 kg 줄었습니다. 이후 이번 하부 업데이트에서 위 0.1183 kg을 추가 반영했습니다.
새 실측이 종전 2.460 kg보다 0.068 kg 늘었다는 사실과 모순되지 않습니다.

중요한 포함 범위 가정
1. L_leg 10.42 kg에는 새 Body가 포함되어 있지 않습니다.
2. 배터리 3.100 kg, 하부 나머지 0.642 kg, 윗파트 전체 2.528 kg은 겹치지 않습니다.
   killSwitch 0.133 kg 및 carrier/board/receiver 0.806 kg는 윗파트 내부 소계입니다.
3. 기존 506/109 g 묶음에서 추정했던 부품별 질량은 새 직접 측정값으로 대체했습니다.
   배터리 3.1 kg은 아래파트이며 한 번만 더합니다.
4. GV7 본체와 긴 USB 케이블은 모두 아래파트 질량이라고 사용자가 확인했습니다.
   PC로 올라가는 선이지만 윗파트 2.528 kg에서 빼거나 그 안에 다시 포함하지 않습니다.
5. GV7 본체 약 20 g과 USB 약 42 g의 합이 실측 묶음 62 g입니다. 62 g을 추가로 더하지 않습니다.
   이전 제조사 명목 17.7 g은 참고 이력으로만 남깁니다.
   이전 참고 출처: {IMU_SOURCE_URL} (2026-10-01 확인)

구조 구분
윗파트: NatureHop_pccarrier와 PC/보드/버튼/killSwitch 및 해당 케이블 등.
아래파트: body_1_tethered_verHopping1, 그 안의 battery_16S_1,
          그 아래의 GV7_ASY_SingleBody IMU, 해당 USB 선, EtherCAT/허브 및 연결선.
기존 STEP의 배터리/IMU 장착 위치는 유지하며 메시 형상은 변경하지 않았습니다.

하부 케이블의 임시 질량 분포
GV7 본체 약 20 g은 IMU CAD 형상에만 적용합니다. 케이블 약 42 g을 그 위치에 몰지 않습니다.
GV7 USB 약 42 g은 GV7 CAD 중심부터 P360 CAD 중심까지 균일한 가는 선분으로 근사합니다.
허브-EtherCAT 연결선 약 50 g도 두 기기의 CAD 중심 사이 균일 선분으로 근사합니다.
CAD 중심은 연결점의 대용값이며 실제 포트 위치가 아닙니다. 배선 경로, 케이블 길이,
남는 선을 묶은 위치, 커넥터 질량 분포를 실측한 모델이 아닙니다.
따라서 두 선의 무게중심은 각 대용 선분의 중간, 중앙 관성은
  I = m/12 * ((d dot d)*identity - outer(d,d))
로 계산합니다. d는 두 대용 끝점 사이 벡터입니다. 실제 배선/묶음 사진을 받으면
이 임시 분포를 실제 배치에 맞춰 보정할 수 있습니다.
계산과 끝점: calibration/lower_cable_proxies.json.

관성 모델: 실측 질량 + CAD 형상/장착 배치에 근거한 1차 추정
PC는 Lenovo ThinkStation P360 Tiny의 제조사 표기 1.400 kg을 임시 적용했습니다.
이 값은 최대 구성 기준의 근사값이며 실제 PC 실측값이 아닙니다.
사용자가 P360이라고 확인했고, CAD 외곽 약 186.06 x 43.5 x 187.10 mm가
Tiny 형태에 가까워 Tiny로 추정했습니다. 공식 외형은 179 x 182.9 x 37 mm이며,
CAD 치수와 정확히 같지는 않습니다. 정확한 제품/구성은 라벨이나 실측으로 확인해야 합니다.
출처: {PC_SOURCE_URL} (2026-10-01 확인)
제조사 공개 COM/관성값은 찾지 못했습니다. PC의 COM과 관성은 기존 CAD의
균일 질량분포를 사용하고, 관성 크기에 1.400 kg을 적용한 추정값입니다.
윗파트 실측 총질량 2.528 kg을 적용합니다. 0.806 kg 소계와 killSwitch/cables 0.133 kg는 이 안에 포함된
측정으로 취급하며 추가로 더하지 않습니다. 수신기는 사용자 요청대로 보드에 합쳤습니다.
NatureHop_pccarrier: 직전 추정 0.614541185 kg -> 실측 차분 0.687 kg.
nature_hopper_powerboard + receiver: 직전 보드 추정 0.433413280 kg -> 실측 묶음 0.119 kg.
캐리어 COM/관성은 CAD 형상에 0.687 kg을 균일한 유효 밀도로 적용했습니다.
PLA 출력물의 실제 벽 두께와 infill 분포는 아직 반영하지 않은 추정입니다.
수신기는 별도 CAD를 만들지 않고 보드와 동일한 위치/분포로 근사합니다.
남은 0.189 kg은 버튼, 배선, 기타 미계상 항목과 PC 명목질량 오차가 섞인
질량 예산입니다. 전부 버튼 질량이라고 지정하지 않았습니다.
이 잔여 질량은 상부 PC/캐리어/보드/버튼/killSwitch CAD의 부피비로 공간적으로 분산한
별도 추정 항목입니다. 측정된 캐리어 0.687 kg과 보드/수신기 0.119 kg은 유지합니다.
killSwitch 관련 케이블은 기존 윗파트 모델에 포함합니다. 하부 두 케이블은 위 선분 분포로 분리했습니다.
각 부품 COM 중심 관성을 회전 변환하고 평행축 정리로 합성했습니다.
OCP 관성 행렬은 물리 텐서이므로 SolidWorks 관성곱처럼 부호를 뒤집지 않았습니다.

이번 변경: 윗파트 3축 COM 실측 보정 (2026-10-02)
두 측정 모두 지지 간격을 약 200 mm로 정정했습니다. 이전 좌우 185 mm 값은 폐기합니다.
좌우: +Y/killSwitch 쪽 1377.5 g, -Y 쪽 1149.0 g, 합 2526.5 g.
앞뒤: 물체를 실제로 90도 회전시켜 측정. -X/killSwitch 쪽 1329.0 g,
      +X 쪽 1179.0 g, 합 2508.0 g.
각 축: COM = 지지 중간 좌표 + D/2 * (양의 방향 반력 - 음의 방향 반력) / 반력 합.
좌우/앞뒤는 지지 중간이 캐리어 CAD 중심축에 놓이고, 유효 지지선 간격이 200 mm라고 가정했습니다.
CAD 림과 양립하는 근사 배치이지만, 넓은 받침 위 실제 접촉선은 별도 검증하지 않았습니다.
지지 중간이 실제로 어긋나 있으면 그 오차가 계산된 COM에 그대로 더해집니다.
간격 1 mm 오차당 X 오프셋은 약 0.0299 mm, Y 오프셋은 약 0.0452 mm 변합니다.
윗파트(2.528 kg) COM [Body_Nature 축, mm]:
  기존 CAD 추정: {vector(upper_cad_com*1000)}
  보정 후:       {vector(upper_calibrated_com*1000)}
따라서 X 약 -6.0 mm, Y 약 +9.0 mm이며, 이번에 추가된 Z는 다음 계산을 사용합니다.

높이 방향: 물체를 옆으로 세워 윗면/아랫면 쪽 모서리를 각각 지지했습니다.
사진 왼쪽=윗면(+Z) 1273.8 g, 오른쪽=아랫면(-Z) 1257.5 g.
사용자가 알려준 지지 간격 100 mm를 사용합니다. 반력 합은 2531.3 g입니다.
아랫 지지선 기준 COM 높이 = 100 * 1273.8 / 2531.3 = 50.321969 mm.
즉 두 지지선 중간에서 윗면 방향으로 약 0.322 mm입니다.
절대 좌표 변환에는 접촉 위치 가정이 필요합니다. CAD 외곽 높이는 105 mm이지만,
아랫 모서리 재료는 Body Z=110~115 mm, 윗 모서리는 210~215 mm 범위입니다.
각 5 mm 두께의 중앙인 112.5/212.5 mm를 명목 지지선으로 잡으면 간격이 100 mm입니다.
따라서 윗파트 절대 COM Z = 112.5 + 50.321969 = 162.821969 mm로 근사 적용합니다.
실제 접촉 압력 중심은 확인하지 못했으므로 이 기준점 선택을 확정 실측이라고 하지 않습니다.
두 반력선이 100 mm 간격을 유지하며 각 5 mm 모서리 안에서 이동한다고 보면,
절대 Z는 명목값에서 +/-2.5 mm 달라질 수 있습니다. 이는 기하학적 민감도이며,
저울/간격/자세/케이블 오차까지 포함한 통계적 신뢰구간은 아닙니다.
CAD 장착면 Z=110 mm에 50.322 mm를 바로 더하면 160.322 mm가 되지만,
그렇게 하면 아래 외측면과 위 안쪽면을 각각 지지선으로 삼는 비대칭 가정이 됩니다.
이번 모델은 같은 두께 모서리를 유사하게 지지했다고 보고 중앙선 기준을 채택했습니다.

반력 합이 별도 실측 2528 g과 좌우/앞뒤/높이 각각 -1.5 g/-20 g/+3.3 g 차이 나므로
반력은 COM 비율에만 사용하고 질량은 2.528 kg을 유지했습니다.
상부 부품별 실제 COM/질량분포를 알아낸 것은 아닙니다. 상부 전체의 XYZ 1차 모멘트를 보정했습니다.
상부 전체의 COM 중심 관성은 기존 CAD 합성 추정값을 유지하고,
보정된 상부 COM 및 하부/배터리/IMU 분포를 평행축 정리로 다시 합쳐
Body_Nature 전체 COM와 관성 6성분을 갱신했습니다. 관성을 실측한 것은 아닙니다.
개별 부품 CAD 좌표나 visual/collision/관절 위치는 변경하지 않았습니다.
calibration/nature_mass_model.json의 body_components는 상부 COM 보정 전 CAD/하부 케이블 근거이며,
최종 재계산에는 upper_com_calibration의 집합 보정을 함께 적용해야 합니다.
원시 수치와 가정: calibration/upper_com_calibration.json.

Body_Nature COM [m]: {vector(body_com)}
Body_Nature COM-centered inertia [kg m^2]:
{np.array2string(body_I,precision=12)}

좌표 및 메시
새 STEP에서 Body 원점의 Thigh CAD 상대 위치는 (0,-147.7,0) mm입니다.
기존 CAD->Thigh URDF 변환 [X,Y,Z]->[Z,X,Y], translation=(0,0,147.7) mm로
새 Body 원점은 Thigh URDF 원점에 놓입니다. 새 STEP의 Calf 원점도 기존
knee origin=(0,0,-252.3) mm와 일치하는 것을 확인했습니다.
배포 DAE의 정점은 이미 위 회전을 적용한 미터 단위입니다.
따라서 joint/visual/inertial axes 모두 rpy=(0,0,0)로 사용합니다.
기존 dummy 무게추의 joint z=0.1 m를 새 Body에 복사하지 않았습니다.
새 Body collision은 높이별 보수적인 box 3개입니다. 내부 빈 공간 일부를 채웁니다.

기존 모델에서 유지한 사항
10개 다리 링크의 COM, visual/collision, 6.35 mm 밑창 설정을 유지했습니다.
다리 질량 및 COM 관성 6성분에만 같은 1.000799913507194 배율을 적용했습니다.
관절 위치/축/제한, 모터 토크·속도, rotor_inertia 값은 유지했습니다.
기존 Thigh 재분할 등 미해결 모델 가정은 이번 업데이트로 해결된 것이 아닙니다.
임시 Thigh +200g 버전과 hopper_cam.urdf는 기준으로 사용하지 않았습니다.
고정 Body 링크 이름은 Temp_Weight에서 Body_Nature로,
고정 joint 이름은 09_temp_weight_joint에서 09_body_nature_joint로 바뀌었습니다.
가동 joint 이름과 순서는 그대로입니다. 기존 Body 이름을 사용하는 코드가 있다면
그 참조만 새 이름으로 수정해야 합니다.

검증
XML/메시 경로/질량 합/관성 양의정부호·삼각부등식/원래 관절 설정 유지 확인.
10개 Body CAD 부품, {len(vertices):,} vertices, {len(idx)//3:,} triangles.
세 축 반력 계산 및 상부/하부 관성 재합성을 독립 수치 계산과 대조했습니다.
RaiSim 실행 검증, 전체 Body COM의 독립 검증, 실제 지지선 절대 좌표와 중앙 관성 실측은 수행하지 않았습니다.
자세한 출처와 가정: calibration/nature_mass_model.json
계산 검증: calibration/validation.json
이전 기준 URDF 원문: calibration/source_Hop2_Parallel_20260922.xml
'''
(OUT/'README_Nature_20261001.txt').write_text(readme,encoding='utf-8')

# 2026-10-08 current revision. The preceding stage reconstructs the measured
# 2026-10-02 source model; it is deliberately NOT globally normalized.
from copy import deepcopy
source_root = ET.fromstring(ET.tostring(root))
source_validation = deepcopy(validation)
TARGET_ROBOT_MASS = Decimal('17.100')
CURRENT_UPPER_MASS = Decimal('2.550')
BATTERY_MASS = Decimal('3.100')
INFERRED_LOWER_NONBATTERY = TARGET_ROBOT_MASS-leg_target-CURRENT_UPPER_MASS-BATTERY_MASS
LOWER_RESIDUAL_MASS = INFERRED_LOWER_NONBATTERY-LOWER_NONBATTERY_MASS
CURRENT_LOWER_MASS = INFERRED_LOWER_NONBATTERY+BATTERY_MASS
CURRENT_BODY_MASS = CURRENT_LOWER_MASS+CURRENT_UPPER_MASS
assert INFERRED_LOWER_NONBATTERY == Decimal('1.030')
assert LOWER_RESIDUAL_MASS == Decimal('0.388')
assert CURRENT_BODY_MASS == Decimal('6.680')

# Measured upper total changed by 22 g. Its location is unmeasured: retain
# measured aggregate COM, and update the existing upper residual geometry prior.
masses[UPPER_RESIDUAL_NAME] = float(CURRENT_UPPER_MASS-Decimal('1.4')-Decimal('0.806')-Decimal('0.133'))
upper_residual_status = ('UNLOCALIZED UPPER BUDGET: measured upper 2.550 minus nominal PC 1.400 '
    'minus measured carrier/board/receiver 0.806 minus switch/cables 0.133 = 0.211 kg. '
    'Already inside upper total; includes possible PC mass error. Distribution uses existing '
    'upper CAD volume weights. Additional 22 g location not measured; aggregate COM retained by assumption.')
for c in component_records:
    c['status'] = c['status'].replace('2.528','2.550')
    if c['record_name'] == UPPER_RESIDUAL_NAME:
        c['assigned_mass_kg'] = masses[UPPER_RESIDUAL_NAME]
        c['uniform_geometry_central_inertia_kgm2'] = (masses[UPPER_RESIDUAL_NAME]*unit_tensors[UPPER_RESIDUAL_NAME]).tolist()
        c['status'] = upper_residual_status

# Separate unknown mass, following only the known nonbattery lower distribution.
# Known component values are NOT inflated; battery receives none of the residual.
known_nonbattery_names = [n for n in lower_names if n != 'battery_16S_1']
known_mass, known_com, known_I = combine_distributions([prior_distribution(n) for n in known_nonbattery_names])
assert abs(known_mass-0.642)<1e-12
LOWER_RESIDUAL_NAME = 'lower_unlocalized_residual'
masses[LOWER_RESIDUAL_NAME] = float(LOWER_RESIDUAL_MASS)
centers[LOWER_RESIDUAL_NAME] = R.T@known_com
unit_tensors[LOWER_RESIDUAL_NAME] = R.T@(known_I/known_mass)@R
lower_names.append(LOWER_RESIDUAL_NAME)
lower_residual_status = ('INFERRED UNKNOWN LOWER MASS: 17.100 whole robot - 10.420 leg - '
    '2.550 measured upper - 3.100 battery - 0.642 known nonbattery lower = 0.388 kg. '
    'Not an independently weighed or identified component. Assumed to follow known '
    'nonbattery lower mass distributions proportionally; battery excluded.')
component_records.append({'CAD_name':None,'record_name':LOWER_RESIDUAL_NAME,
    'record_type':'inferred_unlocalized_mass_budget','assigned_mass_kg':float(LOWER_RESIDUAL_MASS),
    'status':lower_residual_status,'uniform_geometry_COM_body_CAD_m':centers[LOWER_RESIDUAL_NAME].tolist(),
    'uniform_geometry_central_inertia_kgm2':(masses[LOWER_RESIDUAL_NAME]*unit_tensors[LOWER_RESIDUAL_NAME]).tolist()})
upper_mass, upper_cad_com, upper_I = combine_distributions([prior_distribution(n) for n in upper_names])
lower_mass, lower_com, lower_I = combine_distributions([prior_distribution(n) for n in lower_names])
total, body_com, body_I = combine_distributions([(upper_mass,upper_calibrated_com,upper_I),(lower_mass,lower_com,lower_I)])
_, cad_only_body_com, cad_only_body_I = combine_distributions([(upper_mass,upper_cad_com,upper_I),(lower_mass,lower_com,lower_I)])
body.find('inertial/origin').set('xyz',vector(body_com))
body.find('inertial/mass').set('value',str(CURRENT_BODY_MASS))
body.find('inertial/inertia').attrib = matrix_attributes(body_I)

# Move outdated body source comments into history rather than presenting them
# as active calibration. Other baseline history is left intact.
for parent in (root,body):
    for node in list(parent):
        if node.tag is ET.Comment and ('NATURE MODEL 2026-10-01' in (node.text or '') or 'Nature body mass =' in (node.text or '')):
            parent.remove(node)
root.insert(0,ET.Comment(f'''
  CURRENT CALIBRATION 2026-10-08: measured whole robot 17.100 kg.
  L_leg 10.420 kg, measured upper INCLUDING switch/cables 2.550 kg,
  measured battery 3.100 kg. Lower EXCLUDING battery inferred by subtraction
  = 1.030 kg, consisting of known 0.642 kg and unknown residual 0.388 kg.
  Complete lower = 4.130 kg; Body_Nature = 6.680 kg.
  Previous whole-robot global multiplier is REMOVED. Battery and measured leg
  are preserved. Residual 0.388 kg is separately recorded and distributed
  proportionally over known NONBATTERY lower mass priors only. This does not
  identify the missing hardware or independently measure the lower assembly.
  Upper internal residual now 0.211 kg is already inside its 2.550 kg total.
  Upper COM retained at {vector(upper_calibrated_com)} m by assumption:
  added 22 g location is unmeasured. Upper central CAD tensor recomputed;
  Body COM and central tensor recombined by the parallel-axis theorem.
  CAD/straight cable priors, finite support-line and Z-datum uncertainties
  remain. No joint/nominal/rotor/limit/collision/mesh changes in this revision.
  See calibration/lower_residual_mass_allocation.json and nature_mass_model.json.
'''))
ET.indent(root,space='    ')
ET.ElementTree(root).write(urdf_path,encoding='utf-8',xml_declaration=True)

allocation = {'date':'2026-10-08','status':'inferred lower total; unknown residual location assumed',
    'accounting_kg':{'whole_robot_measured':17.1,'L_leg_measured':10.42,'upper_measured':2.55,
        'battery_measured':3.1,'lower_excluding_battery_inferred':1.03,'known_nonbattery_lower':known_mass,
        'unknown_lower_residual':0.388,'lower_including_battery':4.13,'Body_Nature':6.68},
    'formula':'17.100 - 10.420 - 2.550 - 3.100 = 1.030; 1.030 - 0.642 = 0.388 kg',
    'allocation_rule':'unknown 0.388 kg follows the known 0.642 kg nonbattery lower spatial distribution proportionally',
    'battery_excluded':True,'known_component_masses_unchanged':True,'independently_measured_lower_total':False,
    'hardware_identity_known':False,
    'distribution_weights':{n:masses[n]/known_mass for n in known_nonbattery_names},
    'residual_proxy_portions_kg':{n:float(LOWER_RESIDUAL_MASS)*masses[n]/known_mass for n in known_nonbattery_names},
    'residual_COM_link_m':known_com.tolist(),
    'residual_central_tensor_link_kgm2':(float(LOWER_RESIDUAL_MASS)*known_I/known_mass).tolist(),
    'scope_caveat':'Assumes measured leg, upper, battery are disjoint, complete and refer to the same assembled robot. Residual can also include measurement/scope discrepancies.'}
normalization = {'date':'2026-10-08','status':'disabled; superseded by explicit lower residual allocation',
    'applied':False,'global_factor':'1','previous_global_factor_historical_only':'1.0245656081485919713',
    'replacement':'lower_residual_mass_allocation.json'}
write_json(CAL/'lower_residual_mass_allocation.json',allocation)
write_json(CAL/'global_mass_normalization.json',normalization)

upper_com_calibration.update({'date':'2026-10-08','measured_upper_mass_kg':2.55,
    'reaction_measurement_reference_mass_kg':2.528,
    'mass_source':'new independent upper total 2.550 kg supersedes 2.528 kg; old reaction ratios retained as COM assumption',
    'upper_prior_COM_link_m':upper_cad_com.tolist(),'upper_COM_change_link_m':(upper_calibrated_com-upper_cad_com).tolist(),
    'upper_central_inertia_link_kgm2':upper_I.tolist(),
    'inertia_status':'recomputed CAD prior with upper residual 0.211 kg; not measured; retain previous aggregate measured COM by assumption',
    'lower_mass_kg':lower_mass,'lower_COM_link_m':lower_com.tolist(),'lower_central_inertia_link_kgm2':lower_I.tolist(),
    'Body_prior_COM_link_m':cad_only_body_com.tolist(),'Body_prior_central_inertia_link_kgm2':cad_only_body_I.tolist(),
    'Body_COM_change_link_m':(body_com-cad_only_body_com).tolist(),
    'modeling_rule':'recompute CAD central tensors with current budgets; retain old upper COM; combine upper/lower by parallel-axis theorem'})
for test in upper_com_calibration['tests'].values():
    test['reaction_sum_minus_original_reference_g'] = test.pop('reaction_sum_minus_reference_g')
    test['reaction_sum_minus_current_upper_mass_g'] = test['reaction_sum_g']-2550
upper_com_calibration['limitations'] = [s.replace('20 g below the independent upper mass','20 g below the historical 2.528 kg independent mass (42 g below new 2.550 kg)') for s in upper_com_calibration['limitations']]
upper_com_calibration['limitations'] += ['Additional upper 22 g spatial location unmeasured; previous upper COM retained as an explicit approximation.',lower_residual_status]
for cable in cable_records.values():
    cable['mass_group'] = 'lower, excluded from measured upper 2.550 kg'
write_json(CAL/'upper_com_calibration.json',upper_com_calibration)
write_json(CAL/'lower_cable_proxies.json',cable_records)

model.update({'date':'2026-10-08','revision':'Upper measured 2.550; battery 3.100; leg 10.420; inferred nonbattery lower 1.030; explicit lower residual 0.388; robot 17.100. Global scaling removed.',
    'source_stage_assumptions_20261002':model['assumptions'],
    'global_mass_normalization':normalization,'lower_residual_mass_allocation':allocation,
    'source_records_notice':'Current component records sum to Body 6.680 kg before upper COM correction. No global multiplier. Upper 2.550 kg independently weighed; lower 4.130 kg inferred. Historical records are labelled separately.',
    'Body_COM_link_m':body_com.tolist(),'Body_central_inertia_link_kgm2':body_I.tolist(),
    'Body_principal_moments_kgm2':np.linalg.eigvalsh(body_I).tolist()})
model['assumptions'] = [
    'Leg 10.420, upper 2.550 (including switch/cables), battery 3.100, robot 17.100 kg are measured group totals and assumed disjoint where applicable.',
    lower_residual_status,upper_residual_status,
    'Upper COM retains previous three-axis reaction-ratio estimate; support midpoints and Z contact datum remain provisional. Added 22 g location was not measured.',
    'Body components use CAD geometry uniform-density priors, two lower cable segment proxies, and separate residual distributions. Central inertia is estimated, not experimentally identified.',
    'PC 1.400 kg remains a provisional maximum-configuration Tiny specification, not an individual measurement. Upper measured total constrains combined mass.',
    'Known lower 0.642 kg includes approximate measurements; inferred residual inherits their errors and any scope mismatch.',
    'Earlier leg-only 1.0007999135 correction to the measured 10.420 kg remains; whole-robot global 1.0245656081 multiplier is removed.',
    'Receiver remains within measured board group 0.119 kg; carrier 0.687 kg; switch/cables 0.133 kg included once. Prior 0.506/0.109 lower group records are superseded.']
model['mass_kg'].update({'current_Body_component_group_sum':'6.680','known_Body_component_group_sum_before_lower_residual':'6.292',
    'known_robot_sum_before_lower_residual':'16.712','lower_excluding_battery':'1.030','lower_known_excluding_battery':'0.642',
    'lower_unknown_residual':'0.388','lower_including_battery':'4.130','new_Body_group_sum':'6.680','new_robot_sum':'17.100',
    'prior_revision_Body':'6.424026363091671660','prior_revision_robot':'17.100','revision_mass_change':'0.000','effective_L_leg':'10.420'})
model['measured_Body_groups_kg']['upper_assembly_INCLUDING_killSwitch_and_all_upper_cables'] = 2.55
model['PC_source'].update({'measured_upper_assembly_including_switch_kg':2.55,'non_PC_mass_within_upper_kg':1.15})
model['upper_mass_budget'].update({'measured_total_kg':2.55,'unlocalized_residual_kg':0.211})
for key in list(model['user_confirmed_scope']):
    if '2p528' in key:
        model['user_confirmed_scope'][key.replace('2p528','2p550')] = model['user_confirmed_scope'].pop(key)
write_json(CAL/'nature_mass_model.json',model)
with (CAL/'body_mass_ledger.csv').open('w',newline='',encoding='utf-8-sig') as f:
    w=csv.writer(f);w.writerow(['component_or_mass_budget','assigned_mass_kg','evidence_or_assumption'])
    for c in component_records:w.writerow([c['record_name'],f"{c['assigned_mass_kg']:.12f}",c['status']])

check = ET.parse(urdf_path).getroot()
new_total = sum(Decimal(n.get('value')) for n in check.findall('link/inertial/mass'))
assert abs(new_total-TARGET_ROBOT_MASS)<Decimal('1e-15')
assert abs(upper_mass-2.55)<1e-12 and abs(lower_mass-4.13)<1e-12 and abs(total-6.68)<1e-12
assert masses['battery_16S_1']==3.1 and 'battery_16S_1' not in allocation['distribution_weights']
assert abs(sum(c['assigned_mass_kg'] for c in component_records)-6.68)<1e-12
assert abs(sum(allocation['residual_proxy_portions_kg'].values())-.388)<1e-12
for link in check.findall('link'):
    eig=np.linalg.eigvalsh(matrix_from_urdf(link.find('inertial/inertia')))
    assert eig.min()>0 and eig[-1]<=eig[0]+eig[1]+1e-10
    inertia_checks[link.get('name')]=eig.tolist()
# Only Body_Nature inertial properties differ from the measured source stage.
restored=ET.fromstring(ET.tostring(check))
for tag in ('origin','mass','inertia'):
    restored.find("link[@name='Body_Nature']/inertial/"+tag).attrib = dict(source_root.find("link[@name='Body_Nature']/inertial/"+tag).attrib)
assert [(n.tag,n.attrib,(n.text or '').strip()) for n in restored.iter()] == [(n.tag,n.attrib,(n.text or '').strip()) for n in source_root.iter()]
validation.update({'total_mass_kg':str(new_total),'source_stage_validation_20261002':source_validation,
    'checks':['XML parse and total 17.100 kg; leg 10.420, Body 6.680','upper 2.550 and lower 4.130 closure',
    'battery exactly 3.100 and excluded from lower unknown residual distribution',
    'all component records and separate residuals sum to Body budget without double counting',
    'positive definite link inertias and principal triangle inequalities',
    'only Body inertial values differ from regenerated measured source stage; all other non-comment XML identical'],
    'inertia_eigenvalues_kgm2':inertia_checks,'global_factor':'1','lower_residual_mass_kg':'0.388'})
write_json(CAL/'validation.json',validation)
revision_readme = f'''현재 배포판 — 2026-10-08 윗파트 2550 g / 아래파트 잔여 질량 배정

전체 실측                 17.100 kg
기존 다리 실측            10.420 kg
윗파트 실측                2.550 kg (killSwitch 및 윗파트 케이블 포함)
배터리 실측                3.100 kg
아래파트(배터리 제외)      1.030 kg = 17.100 - 10.420 - 2.550 - 3.100
  확인된 부품/케이블        0.642 kg
  아직 확인되지 않은 차이  0.388 kg
아래파트(배터리 포함)      4.130 kg
Body_Nature 전체           6.680 kg

이제 전 링크에 적용하던 1.0245656081배 보정은 사용하지 않습니다.
배터리는 정확히 3.100 kg, 다리는 기존 실측 기준 10.420 kg입니다.
아래파트 1.030 kg은 독립 실측이 아니라 전체에서 뺀 추정값입니다.
미확인 388 g의 실제 부품/위치는 아직 모릅니다. 별도 질량 항목으로 기록하고,
배터리를 제외한 확인된 아래파트 642 g의 기존 공간 분포에 비례해 배치했습니다.
따라서 실측한 개별 부품 질량 자체를 바꾸지 않았습니다.
기존 GV7 USB 및 EtherCAT 케이블은 아래파트로 한 번만 포함됩니다.

윗파트의 기존 2.528 kg을 2.550 kg으로 수정했습니다. 내부 미배정 예산은
0.189 -> 0.211 kg이며 이는 2.550 kg 안에 포함되어 있습니다.
추가 22 g의 위치는 측정하지 않아 윗파트 기존 실측 COM 추정값을 유지했습니다.
윗파트 중앙 관성은 기존 CAD 분포 가정으로 재계산하고, 전체 Body COM/관성을
평행축 정리로 재합산했습니다. 관성을 직접 실측한 것은 아닙니다.
Body_Nature COM [m]: {vector(body_com)}
Body_Nature 중앙 관성 [kg m^2]:
{np.array2string(body_I,precision=12)}

관절/nominal_config/폐루프/rotor_inertia/구동기 제한/메시/충돌은 유지됩니다.
계산 및 XML 검증은 통과했습니다. 시뮬레이터 구동 검증은 하지 않았습니다.
calibration/body_mass_ledger.csv: 현재 Body 구성 질량과 근거
calibration/lower_residual_mass_allocation.json: 388 g의 추정 및 배치 가정
calibration/upper_com_calibration.json: 과거 반력 측정과 현재 질량/관성
calibration/nature_mass_model.json 및 validation.json: 전체 계산/검증

======================================================================
아래는 2026-10-02 원본 단계의 역사 기록입니다. 2.528/3.742/6.270/16.690 kg
등 과거 합계는 위 현재값으로 대체되었습니다. 당시 upper 2.528 kg은 조립체
직접 실측값이었습니다. 아래의 '상하부 모두 별도 실측 아님' 표현은 정정합니다.
======================================================================

'''
(OUT/'README_Nature_20261001.txt').write_text(revision_readme+readme,encoding='utf-8')
(CAL/'build_nature_package.py').write_bytes(Path(__file__).read_bytes())
archive=OUT.parent/(NAME+'_package.zip')
archive_staging=archive.with_suffix('.zip.tmp')
with ZipFile(archive_staging,'w',ZIP_DEFLATED,compresslevel=6) as z:
    for p in sorted(OUT.rglob('*')):
        if p.is_file():z.write(p,str(Path(NAME)/p.relative_to(OUT)))
with ZipFile(archive_staging) as z:
    assert z.testzip() is None
archive_staging.replace(archive)
print(json.dumps({'urdf':str(urdf_path),'zip':str(archive),'body_mass_kg':str(CURRENT_BODY_MASS),'robot_mass_kg':str(new_total),
    'body_COM_m':body_com.tolist(),'body_I_kgm2':body_I.tolist(),'zip_bytes':archive.stat().st_size,'validation':'passed'},indent=2))
