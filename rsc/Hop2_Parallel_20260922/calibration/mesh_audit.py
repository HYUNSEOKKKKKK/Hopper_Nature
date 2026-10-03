from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial import cKDTree

BASE=Path('/workspace/scratch/813fa2b7d6a7/recovered/extracted/Hop2_Parallel_20260820')
NS={'c':'http://www.collada.org/2005/11/COLLADASchema'}

def read(path):
    root=ET.parse(path).getroot()
    geo=root.find('.//c:geometry',NS)
    src=geo.find('.//c:source[@name="position"]/c:float_array',NS)
    if src is None:
        src=geo.find('.//c:source/c:float_array',NS)
    v=np.fromstring(src.text,sep=' ').reshape(-1,3)
    tri=geo.find('.//c:triangles',NS)
    inp=tri.findall('c:input',NS)
    stride=1+max(int(i.get('offset','0')) for i in inp)
    offset=int(next(i for i in inp if i.get('semantic')=='VERTEX').get('offset','0'))
    f=np.fromstring(tri.find('c:p',NS).text,sep=' ',dtype=int).reshape(-1,stride)[:,offset].reshape(-1,3)
    n=np.cross(v[f[:,1]]-v[f[:,0]],v[f[:,2]]-v[f[:,0]])
    n/=np.maximum(np.linalg.norm(n,axis=1,keepdims=True),1e-30)
    return root,v,f,n

def fit_circle_ransac(points,rlo,rhi,tol=0.02,iterations=10000):
    points=np.unique(np.round(points,8),axis=0)
    rng=np.random.default_rng(12345)
    best=None
    for _ in range(iterations):
        p=points[rng.choice(len(points),3,replace=False)]
        A=2*(p[1:]-p[0]);b=(p[1:]**2).sum(1)-(p[0]**2).sum()
        if abs(np.linalg.det(A))<1e-10:continue
        c=np.linalg.solve(A,b);r=np.linalg.norm(p[0]-c)
        if not rlo<=r<=rhi:continue
        errs=abs(np.linalg.norm(points-c,axis=1)-r)
        sel=errs<tol
        score=(sel.sum(),-np.median(errs[sel]))
        if best is None or score>best[0]:best=(score,c,r,sel)
    _,c,r,sel=best
    for _ in range(5):
        p=points[sel];A=np.column_stack([2*p,np.ones(len(p))]);b=(p*p).sum(1)
        sol=np.linalg.lstsq(A,b,rcond=None)[0];c=sol[:2];r=np.sqrt(sol[2]+sum(c*c))
        err=np.linalg.norm(points-c,axis=1)-r
        sel=abs(err)<tol
    return c,r,points[sel],err[sel]

if __name__=='__main__':
    for path in sorted(BASE.glob('URDF_Calf_Link_*.dae')):
        if 'Input' in path.name: continue
        root,v,f,n=read(path)
        print('\n',path.name)
        print('unit',ET.tostring(root.find('c:asset',NS),encoding='unicode'))
        print('scene',ET.tostring(root.find('c:library_visual_scenes',NS),encoding='unicode'))
        print('n_vertices',len(v),'bounds mm',v.min(0)*1000,v.max(0)*1000)
        for axis in range(3):
            values,counts=np.unique(np.round(v[:,axis]*1000,5),return_counts=True)
            idx=np.argsort(counts)[-12:][::-1]
            print('frequent planes',axis,[(float(values[i]),int(counts[i])) for i in idx])
