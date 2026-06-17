import bpy, numpy as np, math
from mathutils.kdtree import KDTree

RIB_BASE = 0.00051   # current base radius
TOE_SCALE = 2.55
OFFSET_SUM = 0.0009 + 0.0009   # total wall gap between the two families

def pts(obj):
    out=[]
    for sp in obj.data.splines:
        for p in sp.points:
            out.append((p.co[0],p.co[1],p.co[2]))
    return np.array(out, dtype=float)

# base verts + gradient weights for radius lookup
boot=bpy.data.objects['left boot cutout meters']
V=np.array([tuple(v.co) for v in boot.data.vertices],dtype=float)
vg=boot.vertex_groups['gradient']; vgi=vg.index
w=np.zeros(len(V))
for v in boot.data.vertices:
    for g in v.groups:
        if g.group==vgi: w[v.index]=g.weight; break
kdw=KDTree(len(V))
for i,c in enumerate(V): kdw.insert(c,i)
kdw.balance()

def radius_at(p):
    co,idx,d=kdw.find((float(p[0]),float(p[1]),float(p[2])))
    wt=w[idx]
    return RIB_BASE*(1.0+(TOE_SCALE-1.0)*wt)

po=pts(bpy.data.objects['Lattice_OUTER'])
pi=pts(bpy.data.objects['Lattice_INNER'])
kdi=KDTree(len(pi))
for i,c in enumerate(pi): kdi.insert(c,i)
kdi.balance()

# for each outer pt: nearest inner pt, local radii, gap
gaps=[]
for c in po:
    co,idx,d=kdi.find(c)
    d_mm=d*1000.0
    ro=radius_at(c); ri=radius_at(pi[idx])
    sumr=(ro+ri)*1000.0
    gap=d_mm - sumr
    gaps.append((d_mm, sumr, gap))
gaps=np.array(gaps)

# bucket by location: weight low (ankle/vented) vs high (toe)
def weight_of(p):
    co,idx,d=kdw.find((float(p[0]),float(p[1]),float(p[2]))); return w[idx]
wp=np.array([weight_of(c) for c in po])
low=wp<0.45   # ankle/vented
mid=(wp>=0.45)&(wp<0.7)
high=wp>=0.7  # toe

def stats(mask, label):
    g=gaps[mask]
    if len(g)==0: return
    gap=g[:,2]
    print('%-22s n=%5d  sep=%.2fmm  sumRadii=%.2fmm  gap min/mean=%.2f/%.2fmm  -> %s' % (
        label, len(g), g[:,0].mean(), g[:,1].mean(), gap.min(), gap.mean(),
        'TOUCHING/FUSED' if gap.min()<=0 else 'GAP (not touching)'))

print('=== DO THE TWO FAMILIES TOUCH? (measured, current 15%%-down values) ===')
print('base radius %.2fmm, toe x%.2f, wall gap(offset) %.2fmm\n' % (RIB_BASE*1000, TOE_SCALE, OFFSET_SUM*1000))
stats(low,  'ankle/vented (w<0.45)')
stats(mid,  'transition (0.45-0.7)')
stats(high, 'toe (w>=0.7)')
print()
print('TOUCH/FUSE begins where sumRadii >= separation (%.2fmm)' % (gaps[:,0].min()))
