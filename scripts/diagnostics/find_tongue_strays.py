import bpy, numpy as np, math

obj = bpy.data.objects['Tongue']
crv = obj.data

# reconstruct the grid to check sample() validity
boot = bpy.data.objects['left boot cutout meters']
mesh = boot.data
# just look at the actual lattice points and check which are near side edges
print('=== ALL lattice segments sorted by max |y| (closest to side edges first) ===')
lat = []
for i, sp in enumerate(crv.splines):
    rad = sp.points[0].radius if sp.points else 1.0
    if rad > 1.5: continue
    pts = np.array([(p.co[0], p.co[1], p.co[2]) for p in sp.points])
    max_abs_y = np.max(np.abs(pts[:,1])) * 1000
    lat.append((max_abs_y, i, len(pts), pts))

lat.sort(reverse=True)
print('top 12 segments by max|y|:')
for may, i, n, pts in lat[:12]:
    # check the endpoints - are they at the boundary?
    ep0 = pts[0]; ep1 = pts[-1]
    # check for any consecutive pair with large jump
    segs = np.diff(pts, axis=0)
    seglens = np.sqrt((segs**2).sum(axis=1))*1000
    max_jump = seglens.max() if len(seglens) else 0
    print('sp[%d] n=%d max|y|=%.1fmm maxjump=%.1fmm  start(%.1f,%.1f) end(%.1f,%.1f)' % (
        i, n, may, max_jump, ep0[0]*1000, ep0[1]*1000, ep1[0]*1000, ep1[1]*1000))

# Now check the actual rim shape vs lattice clip near the sides
# The superellipse: u = cos(t)^(1/n), v = sin(t)^(1/n)
# At what v does the outer rim sit?
print('\n=== Rim boundary shape near sides ===')
for t_deg in [60, 70, 75, 80, 85, 89]:
    t = math.radians(t_deg)
    n = 2.5
    u = math.copysign(abs(math.cos(t))**(1.0/n), math.cos(t))
    v = math.copysign(abs(math.sin(t))**(1.0/n), math.sin(t))
    # lattice clip boundary at same angle
    u_clip = math.copysign(abs(math.cos(t))**(1.0/(2*n)), math.cos(t))
    v_clip = math.copysign(abs(math.sin(t))**(1.0/(2*n)), math.sin(t))
    print('t=%d: rim(u=%.3f,v=%.3f)  clip(u=%.3f,v=%.3f)  gap_v=%.3f' % (
        t_deg, u, v, u_clip, v_clip, v - v_clip))
