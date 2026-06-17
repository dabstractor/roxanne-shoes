import bpy, bmesh, numpy as np
import math

boot = bpy.data.objects['left boot cutout meters']
mesh = boot.data

# --- find the ankle opening (the cuff boundary loop) ---
bm = bmesh.new(); bm.from_mesh(mesh)
open_edges = [e for e in bm.edges if len(e.link_faces) < 2]
# separate the ankle cuff loop from the V edges:
# ankle cuff = the loop at x < -0.005 (the original opening), vs V edges which span x -0.0115..0.078
ankle_verts = []
v_verts = []
for e in open_edges:
    for v in e.verts:
        if v.co.x < -0.006:
            ankle_verts.append((v.co.x*1000, v.co.y*1000, v.co.z*1000))
        elif v.co.x < 0.079:
            v_verts.append((v.co.x*1000, v.co.y*1000, v.co.z*1000))
ankle_verts = np.array(ankle_verts) if ankle_verts else np.zeros((0,3))
print('=== ANKLE OPENING (cuff loop) ===')
if len(ankle_verts):
    print('  verts: %d' % len(ankle_verts))
    print('  x: %.1f..%.1fmm  y: %.1f..%.1fmm  z: %.1f..%.1fmm' % (
        ankle_verts[:,0].min(), ankle_verts[:,0].max(),
        ankle_verts[:,1].min(), ankle_verts[:,1].max(),
        ankle_verts[:,2].min(), ankle_verts[:,2].max()))
    print('  mean x=%.1f  y=%.1f  z=%.1f' % (ankle_verts[:,0].mean(), ankle_verts[:,1].mean(), ankle_verts[:,2].mean()))
bm.free()

# --- find the tongue's back edge position (after rotation) ---
tongue = bpy.data.objects['Tongue']
crv = tongue.data
# the back edge = lowest-x points of the rim
all_pts = []
for sp in crv.splines:
    for p in sp.points:
        all_pts.append((p.co[0]*1000, p.co[1]*1000, p.co[2]*1000))
all_pts = np.array(all_pts)
print('\n=== TONGUE current position ===')
print('  x: %.1f..%.1fmm  y: %.1f..%.1fmm  z: %.1f..%.1fmm' % (
    all_pts[:,0].min(), all_pts[:,0].max(),
    all_pts[:,1].min(), all_pts[:,1].max(),
    all_pts[:,2].min(), all_pts[:,2].max()))
back_pts = all_pts[all_pts[:,0] < all_pts[:,0].min()+2.0]
print('  back edge (lowest x): x %.1f..%.1f  z %.1f..%.1f' % (
    back_pts[:,0].min(), back_pts[:,0].max(), back_pts[:,2].min(), back_pts[:,2].max()))

# the build plate orientation: ankle down. The "flush" plane is the ankle opening's extent.
# When printing ankle-down, the lowest part touches the plate.
# The tongue back should not extend BELOW (more -x than) the ankle opening.
print('\n=== ALIGNMENT ANALYSIS ===')
if len(ankle_verts):
    # which direction is "down" when printing? ankle opening x min
    ankle_xmin = ankle_verts[:,0].min()
    tongue_xmin = all_pts[:,0].min()
    print('  ankle opening min-x: %.1fmm' % ankle_xmin)
    print('  tongue back min-x:   %.1fmm' % tongue_xmin)
    diff = tongue_xmin - ankle_xmin
    if diff > 0:
        print('  -> tongue back is %.1fmm INSIDE the ankle opening (could extend %.1fmm more to be flush)' % (diff, diff))
    else:
        print('  -> tongue back extends %.1fmm PAST the ankle opening (sticks out, needs support)' % abs(diff))
