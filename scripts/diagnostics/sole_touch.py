import bpy, numpy as np
from mathutils.kdtree import KDTree

RIB_BASE=0.00051; TOE_SCALE=2.55
OFFSET_EACH=0.000868   # current
MELT=0.0001            # required melt (0.1 mm) at EVERY crossing

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

def pts(obj):
    out=[]
    for sp in obj.data.splines:
        for p in sp.points: out.append((p.co[0],p.co[1],p.co[2]))
    return np.array(out,dtype=float)

po=pts(bpy.data.objects['Lattice_OUTER']); pi=pts(bpy.data.objects['Lattice_INNER'])
kdi=KDTree(len(pi))
for i,c in enumerate(pi): kdi.insert(c,i)
kdi.balance()

# for each OUTER pt, find nearest INNER pt -> a crossing proxy. sep scales linearly with offset.
cross=[]
for c in po:
    co,idx,d=kdi.find(c)
    ro=radius_at(c); ri=radius_at(pi[idx])
    cross.append((d*1000.0, (ro+ri)*1000.0))
cross=np.array(cross)

# location (sole = low weight) classification via nearest base vert
loc=np.array([w[kdw.find((float(c[0]),float(c[1]),float(c[2])))[1]] for c in po])
def report(mask,label):
    if mask.sum()==0: 
        print('  %-18s n=0'%label); return
    sep=cross[mask,0]; sr=cross[mask,1]; gap=sep-sr
    print('  %-18s n=%5d  sep min/mean/max=%.2f/%.2f/%.2f  gap min=%.3f mean=%.3f' % (
        label,mask.sum(),sep.min(),sep.mean(),sep.max(),gap.min(),gap.mean()))

print('=== CURRENT crossing separation (offset each=%.3fmm) ===' % (OFFSET_EACH*1000))
print('SOLE = pad region (weight<0.35), the problem area\n')
report(loc<0.35,'SOLE/pad (w<.35)')
report((loc>=0.35)&(loc<0.7),'transition')
report(loc>=0.7,'toe (fuses ok)')

# solve: sep scales with offset. For 0.1mm melt at the WORST sole crossing:
#   k * sep_worst <= sumradii_worst - 0.1   =>   k = (sr_worst-0.1)/sep_worst
sole=loc<0.35
sep_sole=cross[sole,0]; sr_sole=cross[sole,1]
# worst = crossing needing most offset reduction (smallest sr/sep ratio)
ratio=(sr_sole-MELT*1000)/sep_sole   # per crossing
k_all=ratio.min()                     # guarantees melt everywhere
new_offset=OFFSET_EACH*k_all
print('\n=== SOLUTION ===')
print('scale factor k to melt ALL sole crossings by >=0.1mm: %.3f' % k_all)
print('current offset each: %.4f mm' % (OFFSET_EACH*1000))
print('NEW offset each:     %.4f mm  (wall %.2fmm)' % (new_offset*1000, 2*new_offset*1000))
print('worst sole crossing: sep=%.2fmm sumradii=%.2fmm -> after scale: overlap=%.2fmm' % (
    sep_sole[ratio.argmin()], sr_sole[ratio.argmin()], sr_sole[ratio.argmin()]-MELT*1000))
print('\n>>> set OFFSET_OUT = OFFSET_IN = %.6f' % new_offset)
