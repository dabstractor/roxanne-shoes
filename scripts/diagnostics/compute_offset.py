import bpy, numpy as np, math
from mathutils.kdtree import KDTree

RIB_BASE = 0.00051
TOE_SCALE = 2.55
BOOT=bpy.data.objects['left boot cutout meters']
V=np.array([tuple(v.co) for v in BOOT.data.vertices],dtype=float)
vg=BOOT.vertex_groups['gradient']; vgi=vg.index
w=np.zeros(len(V))
for v in BOOT.data.vertices:
    for g in v.groups:
        if g.group==vgi: w[v.index]=g.weight; break
kdw=KDTree(len(V))
for i,c in enumerate(V): kdw.insert(c,i)
kdw.balance()

def radius_at(p):
    co,idx,d=kdw.find((float(p[0]),float(p[1]),float(p[2])))
    return RIB_BASE*(1.0+(TOE_SCALE-1.0)*w[idx])

def pts(obj):
    out=[]
    for sp in obj.data.splines:
        for p in sp.points: out.append((p.co[0],p.co[1],p.co[2]))
    return np.array(out,dtype=float)

po=pts(bpy.data.objects['Lattice_OUTER'])
pi=pts(bpy.data.objects['Lattice_INNER'])

# for each outer pt, scale its offset from the base surface by factor f.
# We don't have base-surface proximity stored, so instead measure:
# current total sep at each pt, and compute f so that sep*f = sumRadii.
kdi=KDTree(len(pi))
for i,c in enumerate(pi): kdi.insert(c,i)
kdi.balance()

records=[]
for c in po:
    co,idx,d=kdi.find(c)
    sep=d*1000.0
    sr=(radius_at(c)+radius_at(pi[idx]))*1000.0
    # f = sr/sep makes them exactly touch at this crossing
    if sep>1e-9:
        records.append((sep, sr, sr/sep))
records=np.array(records)
# ankle = where weight is low
wp=np.array([w[kdw.find((float(c[0]),float(c[1]),float(c[2])))[1]] for c in po])
ankle=wp<0.45
rec_a=records[ankle]
# offset scales linearly with sep. Current offset total=1.8mm -> sep ~1.9mm.
# new offset total = current * f_mean. Use f that makes the WORST (largest) ankle gap -> 0
f_needed = (rec_a[:,1]/rec_a[:,0]).max()   # most-negative-gap crossing sets the touch point
cur_offset_total=0.0009+0.0009
new_offset_total=cur_offset_total*f_needed
new_each=new_offset_total/2.0
print('=== COMPUTE OFFSET FOR JUST-TOUCHING AT ANKLE ===')
print('current ankle: sep min/mean=%.2f/%.2fmm  sumRadii min/mean=%.2f/%.2fmm' % (
    rec_a[:,0].min(), rec_a[:,0].mean(), rec_a[:,1].min(), rec_a[:,1].mean()))
print('current gap min/mean=%.2f/%.2fmm (positive=gap)' % (
    (rec_a[:,0]-rec_a[:,1]).min(), (rec_a[:,0]-rec_a[:,1]).mean()))
print('scale factor needed: %.3f  (1.0=current, <1=bring closer)' % f_needed)
print('current offset EACH side: %.4fmm' % (cur_offset_total*1000/2))
print('NEW offset EACH side:     %.4fmm' % (new_each*1000))
print('wall thickness at ankle:  %.2fmm -> %.2fmm' % (cur_offset_total*1000, new_offset_total*1000))
print()
print('>>> Set OFFSET_OUT = OFFSET_IN = %.5f  in /tmp/build_lattice.py' % new_each)
