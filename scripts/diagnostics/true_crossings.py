import bpy, numpy as np
from mathutils.kdtree import KDTree

RIB_BASE=0.00051; TOE_SCALE=2.55
boot=bpy.data.objects['left boot cutout meters']
mesh=boot.data
V=np.array([tuple(v.co) for v in mesh.vertices],dtype=float)
vg=boot.vertex_groups['gradient']; vgi=vg.index
w=np.zeros(len(V))
for v in mesh.vertices:
    for g in v.groups:
        if g.group==vgi: w[v.index]=g.weight; break
kdw=KDTree(len(V))
for i,c in enumerate(V): kdw.insert(c,i)
kdw.balance()
def radius_at(p):
    co,idx,d=kdw.find((float(p[0]),float(p[1]),float(p[2])))
    return RIB_BASE*(1.0+(TOE_SCALE-1.0)*w[idx])

# sample outer & inner splines as (point, tangent) pairs
def sample(obj, step=1):
    pts=[]; tan=[]
    for sp in obj.data.splines:
        ps=[(p.co[0],p.co[1],p.co[2]) for p in sp.points]
        if len(ps)<2: continue
        for i in range(0,len(ps)-1,step):
            a=np.array(ps[i]); b=np.array(ps[i+1])
            t=b-a; L=np.linalg.norm(t)
            if L<1e-9: continue
            pts.append(a); tan.append(t/L)
    return np.array(pts), np.array(tan)

po,to=sample(bpy.data.objects['Lattice_OUTER'])
pi,ti=sample(bpy.data.objects['Lattice_INNER'])
print('outer samples: %d  inner samples: %d' % (len(po),len(pi)))
kdi=KDTree(len(pi))
for i,c in enumerate(pi): kdi.insert(c,i)
kdi.balance()

# for each outer sample, find nearby inner samples; a TRUE crossing = close + non-parallel tangent
CROSS_COS=0.7   # |cos(tangents)| < 0.7  => angle>45deg => genuine crossing
crossings=[]   # (sep_mm, melt_mm, weight)
for k in range(len(po)):
    res=kdi.find_n(po[k],12)
    for (co,j,d) in res:
        if d>0.003: break            # only local
        c=abs(float(to[k].dot(ti[j])))   # |cos angle|
        if c>CROSS_COS: continue     # parallel => wall proximity, skip
        sep=d*1000.0
        sr=(radius_at(po[k])+radius_at(pi[j]))*1000.0
        melt=sr-sep
        wt=w[kdw.find((float(po[k][0]),float(po[k][1]),float(po[k][2])))[1]]
        crossings.append((sep,sr,melt,wt))
crossings=np.array(crossings)
print('TRUE crossings detected: %d' % len(crossings))

sep=crossings[:,0]; melt=crossings[:,2]; wt=crossings[:,3]
def rep(mask,label):
    m=melt[mask]
    if len(m)==0: print('  %-14s n=0'%label); return
    print('  %-14s n=%5d | melt min=%.3f mean=%.3f mm | <0.1mm melt: %d (%.1f%%) | gapping(<0): %d (%.1f%%)' % (
        label,len(m),m.min(),m.mean(),(m<0.1).sum(),100*(m<0.1).mean(),(m<0).sum(),100*(m<0).mean()))

print('=== TRUE-CROSSING MELT AUDIT (offset each=0.448mm) ===')
rep(wt<0.35,'SOLE/pad')
rep((wt>=0.35)&(wt<0.7),'transition')
rep(wt>=0.7,'toe')
print()
print('sole worst crossing melt: %.3fmm  (need >=0.1)' % melt[wt<0.35].min() if (wt<0.35).sum() else 'n/a')
