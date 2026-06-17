import bpy, bmesh, math, numpy as np
from collections import defaultdict
from mathutils.kdtree import KDTree

boot = bpy.data.objects['left boot cutout meters']
mesh = boot.data
for nm in ('Rims','Rim'):
    old=bpy.data.objects.get(nm)
    if old: bpy.data.objects.remove(old, do_unlink=True)

bm=bmesh.new(); bm.from_mesh(mesh)
open_edges=[e for e in bm.edges if len(e.link_faces)<2]
print('open boundary edges:', len(open_edges))

# --- DIRECTION-AWARE chaining (no backtracking, same technique as lattice) ---
def chain_edges_dir(edges):
    def key(v): return (round(v.co.x,6), round(v.co.y,6), round(v.co.z,6))
    adj=defaultdict(list)
    for e in edges:
        a,b=e.verts
        adj[key(a)].append((key(b),b,e))
        adj[key(b)].append((key(a),a,e))
    used=set(); loops=[]
    for e0 in edges:
        for v in e0.verts:
            k=key(v)
            if k in used: continue
            used.add(k); cur=v; pts=[cur.co.copy()]; prev_dir=None
            while True:
                nbrs=[(kk,vv,ee) for (kk,vv,ee) in adj[key(cur)] if kk not in used and ee.index not in used]
                if not nbrs: break
                if prev_dir is None:
                    kk,vv,ee=nbrs[0]
                else:
                    best=None; best_dot=-2.0
                    for (kk,vv,ee) in nbrs:
                        d=vv.co-cur.co
                        dn=d.length+1e-9
                        dot=d.dot(prev_dir)/(dn*prev_dir.length+1e-9)
                        if dot>best_dot: best_dot=dot; best=(kk,vv,ee)
                    kk,vv,ee=best
                used.add(kk); used.add(ee.index)
                d=vv.co-cur.co
                pts.append(vv.co.copy()); prev_dir=d; cur=vv
                if len(pts)>5000: break
            if len(pts)>=3: loops.append(pts)
    return loops

loops=chain_edges_dir(open_edges)
print('boundary loops (direction-aware):', len(loops))
total=0
for i,l in enumerate(loops):
    pts=np.array([(p.x,p.y,p.z) for p in l])
    segs=np.diff(pts,axis=0); dots=(segs[:-1]*segs[1:]).sum(axis=1)
    n1=np.sqrt((segs[:-1]**2).sum(axis=1)); n2=np.sqrt((segs[1:]**2).sum(axis=1))
    cos=dots/(n1*n2+1e-9)
    rev=int((cos<0).sum())
    total+=len(l)
    xs=pts[:,0]*1000
    print('  loop %d: n=%d  X %.1f..%.1f  reversals=%d' % (i,len(l),xs.min(),xs.max(),rev))
print('total rim points:', total)

# --- smooth rim polylines (kill residual micro-jitter) ---
def smooth(pts,iters):
    if len(pts)<3: return pts
    pts=[p.copy() for p in pts]
    for _ in range(iters):
        nw=pts[:]
        for i in range(1,len(pts)-1):
            nw[i]=(pts[i-1]+pts[i+1])*0.5
        pts=nw
    return pts

RIM_RADIUS=0.0012
RIM_OFFSETS=[+0.0009, 0.0, -0.0009]
N=np.array([tuple(v.normal) for v in mesh.vertices],dtype=float)
base=np.array([tuple(v.co) for v in mesh.vertices],dtype=float)
kd=KDTree(len(base))
for i,c in enumerate(base): kd.insert(c,i)
kd.balance()
def normal_at(p):
    co,idx,d=kd.find(p); return N[idx]

crv=bpy.data.curves.new('Rims','CURVE')
crv.dimensions='3D'; crv.bevel_depth=RIM_RADIUS; crv.bevel_resolution=3; crv.use_fill_caps=True; crv.resolution_u=12
for loop in loops:
    sloop=smooth(loop,5)
    for off in RIM_OFFSETS:
        sp=crv.splines.new('POLY'); sp.use_cyclic_u=True
        sp.points.add(len(sloop)-1)
        for i,p in enumerate(sloop):
            p2=np.array([p.x,p.y,p.z],dtype=float)+normal_at(p)*off
            sp.points[i].co=(float(p2[0]),float(p2[1]),float(p2[2]),1.0)
rim_obj=bpy.data.objects.new('Rims',crv)
bpy.context.collection.objects.link(rim_obj); rim_obj.parent=boot

print('=== RIMS REBUILT (direction-aware + smoothed) ===')
print('reversals should be near 0 now; jaggedness eliminated')
