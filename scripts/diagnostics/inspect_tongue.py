import bpy, numpy as np, math

obj = bpy.data.objects.get('Tongue')
if obj is None:
    print('No Tongue object'); raise SystemExit
crv = obj.data
print('=== TONGUE CURVE: %d splines ===' % len(crv.splines))

# classify each spline
for i, sp in enumerate(crv.splines):
    pts = np.array([(p.co[0], p.co[1], p.co[2]) for p in sp.points])
    if len(pts) == 0: continue
    rad = sp.points[0].radius if sp.points else 1.0
    cyclic = sp.use_cyclic_u
    # bounds
    xr = (pts[:,0].min()*1000, pts[:,0].max()*1000)
    yr = (pts[:,1].min()*1000, pts[:,1].max()*1000)
    zr = (pts[:,2].min()*1000, pts[:,2].max()*1000)
    n = len(pts)
    is_rim = rad > 1.5
    kind = 'RIM' if is_rim else 'lattice'
    cyc = 'cyclic' if cyclic else 'open'
    print('sp[%3d] %-7s %-8s n=%4d rad=%.1f  x=%.0f..%.0f y=%.0f..%.0f z=%.0f..%.0f' % (
        i, kind, cyc, n, rad, xr[0],xr[1], yr[0],yr[1], zr[0],zr[1]))

# focus on RIM splines - are there more than 2? what are they?
print('\n=== RIM splines detail ===')
for i, sp in enumerate(crv.splines):
    rad = sp.points[0].radius if sp.points else 1.0
    if rad > 1.5:
        pts = np.array([(p.co[0], p.co[1], p.co[2]) for p in sp.points])
        print('RIM sp[%d]: %d pts, cyclic=%s' % (i, len(pts), sp.use_cyclic_u))
        print('  x %.1f..%.1f  y %.1f..%.1f  z %.1f..%.1f' % (
            pts[:,0].min()*1000, pts[:,0].max()*1000,
            pts[:,1].min()*1000, pts[:,1].max()*1000,
            pts[:,2].min()*1000, pts[:,2].max()*1000))

# the "extra pieces near attachment" - look for non-rim splines with unusual extent
# attachment is at FRONT_X ~ 82mm. Splines running along the side near there.
print('\n=== lattice splines near attachment (x>70mm) ===')
for i, sp in enumerate(crv.splines):
    rad = sp.points[0].radius if sp.points else 1.0
    if rad > 1.5: continue
    pts = np.array([(p.co[0], p.co[1], p.co[2]) for p in sp.points])
    if pts[:,0].max()*1000 > 70:
        print('sp[%d] n=%d x=%.0f..%.0f y=%.0f..%.0f' % (i, len(pts), pts[:,0].min()*1000, pts[:,0].max()*1000, pts[:,1].min()*1000, pts[:,1].max()*1000))
