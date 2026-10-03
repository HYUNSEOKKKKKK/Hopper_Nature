"""Measure ball center to intersecting eye-end pin axes in STEP link coordinates."""
import json
from pathlib import Path
import numpy as np

root=Path(__file__).parent
records=json.loads((root/'step_local_link_geometry.json').read_text())
result=[]
for assembly in ('Calf_link_S_verU1','Calf_link_L_verU1'):
    subset=[r for r in records if r['path'][0]==assembly and r['surfaces']]
    ball=next(r for r in subset if 'E-GZW8_b_20240529' in r['path'])
    eye=next(r for r in subset if 'Eye_end_20240529' in r['path'])
    sphere=next(s for s in ball['surfaces'] if s['kind']=='SPHERICAL_SURFACE')
    a=next(s for s in eye['surfaces'] if s['kind']=='CYLINDRICAL_SURFACE' and abs(s['radius']-5)<1e-7)
    b=next(s for s in eye['surfaces'] if s['kind']=='CYLINDRICAL_SURFACE' and abs(s['radius']-6)<1e-7)
    pa,pb=np.array(a['center']),np.array(b['center'])
    da,db=np.array(a['axis']),np.array(b['axis'])
    t=np.linalg.lstsq(np.column_stack((da,-db)),pb-pa,rcond=None)[0]
    qa,qb=pa+t[0]*da,pb+t[1]*db
    eye_center=(qa+qb)/2
    assert np.linalg.norm(qa-qb)<1e-7
    ball_center=np.array(sphere['center'])
    out=dict(assembly=assembly,units='mm',eye_center=eye_center.tolist(),ball_center=ball_center.tolist(),
             eye_axis_intersection_residual=float(np.linalg.norm(qa-qb)),
             center_distance=float(np.linalg.norm(ball_center-eye_center)),
             ball_sphere_entity=sphere['entity'],eye_cylinder_entities=[a['entity'],b['entity']])
    result.append(out)
print(json.dumps(result,indent=2))
(root/'step_center_distances.json').write_text(json.dumps(result,indent=2)+'\n')
