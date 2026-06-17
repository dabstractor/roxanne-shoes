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
def pts(obj):
    out=[]
    for sp in obj.data.splines:
        for p in sp.points: out.append((p.co[0],p.co[1],p.co[2]))
    return np.array(out,dtype=float)
po=pts(bpy.data.objects['Lattice_OUTER']); pi=pts(bpy.data.objects['Lattice_INNER'])
kdi=KDTree(len(pi))
for i,c in enumerate(pi): kdi.insert(c,i)
kdi.balance()

cross=[]; loc=[]
for c in po:
    co,idx,d=kdi.find(c)
    ro=radius_at(c); ri=radius_at(pi[idx])
    cross.append((d*1000.0, (ro+ri)*1000.0))
    loc.append(w[kdw.find((float(c[0]),float(c[1]),float(c[2])))[1]])
cross=np.array(cross); loc=np.array(loc)
gap=cross[:,0]-cross[:,1]   # positive = GAP, negative = melt/overlap

def full(mask,label):
    g=gap[mask]
    print('%-18s n=%5d | gap min=%.3f mean=%.3f MAX=%.3f mm | still-gapping(>0): %d (%.2f%%) | melt<0.1mm: %d (%.2f%%)' % (
        label,mask.sum(), g.min(), g.mean(), g.max(),
        (g>0).sum(), 100*(g>0).mean(), (g>-0.1).sum(), 100*(g>-0.1).mean()))

print('=== FULL GAP AUDIT (offset each = 0.448mm) ===')
print('positive gap = NOT touching; melt<0.1mm = touching but under-spec\n')
full(loc<0.35,'SOLE/pad')
full((loc>=0.35)&(loc<0.7),'transition')
full(loc>=0.7,'toe')

# what offset guarantees melt>=0.1 EVERYWHERE on sole? sep scales ~linearly with offset.
# current offset O=0.448. worst sole gap-crossing: need to close its (sep - (sr-0.1)).
sole=loc<0.35
sep=cross[sole,0]; sr=cross[sole,1]
# required scale so that sep_new <= sr - 0.1 for all: k = min over crossings of (sr-0.1)/sep
k=( (sr-0.1)/sep ).min()
# but sep_new = sep * (O_new/O_cur), so O_new = O_cur * k
O_cur=0.000448
O_new=O_cur*k
print('\n=== to guarantee melt>=0.1mm at EVERY sole crossing ===')
print('required offset each: %.4f mm (current 0.448)' % (O_new*1000))
print('  worst crossing: sep=%.2fmm sumradii=%.2fmm' % (sep[((sr-0.1)/sep).argmin()], sr[((sr-0.1)/sep).argmin()]))
print('  wall would be %.2fmm' % (2*O_new*1000))
