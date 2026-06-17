import bpy, numpy as np
from mathutils.kdtree import KDTree

obj = bpy.data.objects['Tongue']
boot = bpy.data.objects['left boot cutout meters']

# get evaluated mesh (with bevel applied) to see actual tubes
dg = bpy.context.evaluated_depsgraph_get()
ev = obj.evaluated_get(dg)
em = ev.to_mesh()
print('evaluated mesh: %d verts' % len(em.vertices))

# the two rim splines define where the sidewalls SHOULD be
# let me find all tubes (connected components) near the left side edge
# first, get the rim centerlines from the curve data
crv = obj.data
rim_pts = []
for sp in crv.splines:
    rad = sp.points[0].radius if sp.points else 1.0
    if rad > 1.5:  # rim
        for p in sp.points:
            rim_pts.append((p.co[0]*1000, p.co[1]*1000, p.co[2]*1000))
rim_pts = np.array(rim_pts)
print('rim centerline points: %d' % len(rim_pts))

# find evaluated verts that are near the left sidewall region (y < -7mm) but NOT on a rim
# build KDTree of rim points
kdr = KDTree(len(rim_pts))
for i, p in enumerate(rim_pts): kdr.insert(p, i)
kdr.balance()

near_side_not_rim = []
for v in em.vertices:
    p = (v.co.x*1000, v.co.y*1000, v.co.z*1000)
    if p[1] < -7.0:  # left side region
        co, idx, d = kdr.find(p)
        if d > 1.5:  # more than 1.5mm from any rim point = NOT a rim tube
            near_side_not_rim.append(p)

print('evaluated verts near left side but NOT on rim: %d' % len(near_side_not_rim))
if near_side_not_rim:
    na = np.array(near_side_not_rim)
    print('  x: %.1f..%.1f  y: %.1f..%.1f  z: %.1f..%.1f' % (
        na[:,0].min(), na[:,0].max(), na[:,1].min(), na[:,1].max(), na[:,2].min(), na[:,2].max()))
    # these belong to the "third rib" - find which lattice spline they came from
    # build KDTree of all lattice centerline points
    lat_pts = []
    lat_owner = []
    for si, sp in enumerate(crv.splines):
        rad = sp.points[0].radius if sp.points else 1.0
        if rad > 1.5: continue
        for p in sp.points:
            lat_pts.append((p.co[0]*1000, p.co[1]*1000, p.co[2]*1000))
            lat_owner.append(si)
    lat_pts = np.array(lat_pts)
    kdl = KDTree(len(lat_pts))
    for i, p in enumerate(lat_pts): kdl.insert(p, i)
    kdl.balance()
    owners = set()
    for p in near_side_not_rim:
        co, idx, d = kdl.find(p)
        if d < 1.0:
            owners.add(lat_owner[idx])
    print('  responsible lattice splines: %s' % sorted(owners))
    # show details of those splines
    for si in sorted(owners)[:5]:
        sp = crv.splines[si]
        pts = np.array([(p.co[0]*1000, p.co[1]*1000, p.co[2]*1000) for p in sp.points])
        print('    sp[%d] n=%d  y: %.1f..%.1f  first3: %s' % (
            si, len(pts), pts[:,1].min(), pts[:,1].max(),
            [(round(p[0]),round(p[1])) for p in pts[:3]]))
ev.to_mesh_clear()
