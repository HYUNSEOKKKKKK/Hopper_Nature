"""Inspect analytic STEP geometry without tessellating the full robot."""
import re, json
from pathlib import Path
from collections import defaultdict
import numpy as np

root=Path(__file__).parent
data=(root/'Hopper_verX2.STEP').read_text()
entities={int(m[1]):m[2].strip() for m in re.finditer(r'#(\d+)\s*=\s*(.*?);',data,re.S)}
refs=lambda text: [int(x) for x in re.findall(r'#(\d+)',text)]
names={k:re.search(r"(?:SHAPE_REPRESENTATION|ADVANCED_BREP_SHAPE_REPRESENTATION)\s*\(\s*'([^']*)'",v) for k,v in entities.items() if 'SHAPE_REPRESENTATION' in v[:100]}
names={k:v[1] for k,v in names.items() if v}
def coords(i):
    s=entities[i]
    vals=re.findall(r'\(\s*([^()]*)\s*\)',s)[-1]
    return np.array([float(v) for v in vals.split(',')])
def placement(i):
    p,z,x=refs(entities[i])
    z=coords(z); x=coords(x)
    z=z/np.linalg.norm(z); x=x-z*(x@z); x=x/np.linalg.norm(x)
    t=np.eye(4); t[:3,:3]=np.column_stack((x,np.cross(z,x),z));t[:3,3]=coords(p)
    return t
adj=defaultdict(list)
for i,s in entities.items():
    m=re.search(r"REPRESENTATION_RELATIONSHIP\s*\(\s*'[^']*'\s*,\s*'[^']*'\s*,\s*#(\d+)\s*,\s*#(\d+)",s)
    if not m: continue
    a,b=map(int,m.groups())
    mt=re.search(r'REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION\s*\(\s*#(\d+)',s)
    t=np.eye(4)
    if mt:
        p,q=refs(entities[int(mt[1])])
        t=placement(p)@np.linalg.inv(placement(q))
    adj[a].append((b,t,i))

def own_surfaces(rep,t):
    found=[]; seen=set(); stack=[rep]
    while stack:
        i=stack.pop()
        if i in seen:continue
        seen.add(i);s=entities[i]
        if s.startswith(('CYLINDRICAL_SURFACE','SPHERICAL_SURFACE')):
            ap=refs(s)[0]; m=t@placement(ap)
            radius=float(s.rsplit(',',1)[1].split(')')[0].strip())
            found.append(dict(entity=i,kind=s.split(' ')[0],radius=radius,center=m[:3,3].tolist(),axis=m[:3,2].tolist()))
        if i!=rep and i in names:continue
        stack.extend(refs(s))
    return found

records=[]
def walk(rep,t,path,trail):
    if rep in trail:return
    path=path+[names.get(rep,str(rep))]
    surfs=own_surfaces(rep,t)
    record=dict(rep=rep,path=path,transform=t.tolist(),surfaces=surfs)
    records.append(record)
    print('/'.join(path),'surfaces=',len(surfs),'origin=',t[:3,3].round(6).tolist())
    for child,tc,rel in adj[rep]:walk(child,t@tc,path,trail|{rep})
for k,n in names.items():
    if n in ['Calf_link_S_verU1','Calf_link_L_verU1']:walk(k,np.eye(4),[],set())
(root/'step_local_link_geometry.json').write_text(json.dumps(records,indent=2))
