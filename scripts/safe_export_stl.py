"""SAFE export (non-destructive): duplicates the curve objects, converts the COPIES
to mesh, joins them, exports STL, then DELETES the copies. Original curves untouched.
Does NOT include the boot base mesh (that's just a reference surface, not part of the print)."""
import bpy, os, numpy as np

project = '/home/dustin/Documents/Models/Roxanne Shoes/'

# unhide boot so we can manipulate (but we won't export it)
bpy.data.objects['left boot cutout meters'].hide_set(False)

# objects to export: lattice + rims + tongue + ankle reinforcement (the actual printable geometry)
curve_objs = [bpy.data.objects[n] for n in ('Lattice_OUTER','Lattice_INNER','Rims','Tongue','Ankle_Reinforce')]

print('=== SAFE EXPORT (working on copies, originals untouched) ===')
copies = []
for o in curve_objs:
    # duplicate object + data
    o_copy = o.copy()
    o_copy.data = o.data.copy()
    o_copy.name = o.name + '_EXPORT'
    # o.copy() carries the object's modifier stack (e.g. Ankle_Reinforce's Solidify);
    # convert(target='MESH') below bakes those modifiers into the exported mesh.
    bpy.context.collection.objects.link(o_copy)
    copies.append(o_copy)

# convert each copy to mesh
for o_copy in copies:
    bpy.ops.object.select_all(action='DESELECT')
    o_copy.select_set(True)
    bpy.context.view_layer.objects.active = o_copy
    bpy.ops.object.convert(target='MESH')
    print('  %s -> mesh (%d verts)' % (o_copy.name, len(o_copy.data.vertices)))

# join all copies into the first one
master = copies[0]
bpy.ops.object.select_all(action='DESELECT')
for o_copy in copies:
    o_copy.select_set(True)
bpy.context.view_layer.objects.active = master
bpy.ops.object.join()
print('  joined export object: %d verts, %d faces' % (len(master.data.vertices), len(master.data.polygons)))

# === FLATTEN THE ANKLE (-X) END INTO A PERFECT PLANE (opening preserved) ===
# The round rim/tongue beads at the ankle bulge to ~-11.3mm and are not flat,
# so the boot rocks when printed ankle-down on the build plate. Slice off
# everything -X of this plane so every cut face lies in x=ANKLE_FLAT_X (one
# flat plane on the build plate). Then cap ONLY individual small tube ends
# (each lattice/rim tube sliced = a tiny closed ring) for solid adhesion --
# NEVER fill any large loop, which is what sealed the foot opening shut in the
# first attempt. The macro foot-opening hole stays open by construction.
# (Non-destructive: this only touches the joined EXPORT copy.)
import bmesh
from collections import defaultdict
ANKLE_FLAT_X = -0.0100   # -10.0mm = flush with the shoe body's -X extent
CAP_MAX_AREA_M2 = 30.0e-6   # cap tube ends up to 30mm^2; opening is ~1500mm^2 -> always skipped
before_v = len(master.data.vertices)
bm = bmesh.new(); bm.from_mesh(master.data)
bmesh.ops.bisect_plane(bm, geom=bm.verts[:] + bm.edges[:] + bm.faces[:],
                       plane_co=(ANKLE_FLAT_X,0,0), plane_no=(1,0,0),
                       dist=1e-7, clear_outer=False, clear_inner=False)
# delete everything strictly -X of the cut plane (trims the beads flat)
bmesh.ops.delete(bm, geom=[v for v in bm.verts if v.co.x < ANKLE_FLAT_X - 1e-6], context='VERTS')
# weld coincident verts the bisect may have created on the plane
bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=2e-6)
bm.verts.ensure_lookup_table(); bm.edges.ensure_lookup_table()

# boundary edges lying exactly on the plane = the cut tube ends (+ rim bead slits)
cut_edges = [e for e in bm.edges
             if len(e.link_faces) == 1
             and abs(e.verts[0].co.x - ANKLE_FLAT_X) < 1e-5
             and abs(e.verts[1].co.x - ANKLE_FLAT_X) < 1e-5]

# group cut edges into connected loops (union-find over shared verts)
parent = {}
def find(x):
    parent.setdefault(x, x)
    while parent[x] != x:
        parent[x] = parent[parent[x]]; x = parent[x]
    return x
def union(a, b):
    ra, rb = find(a), find(b)
    if ra != rb: parent[ra] = rb
for e in cut_edges:
    union(e.verts[0].index, e.verts[1].index)
comps = defaultdict(list)
for e in cut_edges:
    comps[find(e.verts[0].index)].append(e)

n_capped = n_open = n_big = 0
for edgelist in comps.values():
    verts_in = set()
    deg = defaultdict(int)
    adj = defaultdict(list)
    for e in edgelist:
        a, b = e.verts[0].index, e.verts[1].index
        verts_in.add(a); verts_in.add(b); deg[a] += 1; deg[b] += 1
        adj[a].append(b); adj[b].append(a)
    closed = len(edgelist) >= 3 and all(deg[v] == 2 for v in verts_in)
    if not closed:
        n_open += 1; continue          # open ribbon (e.g. rim bead slit) -> leave flat-cut, no cap
    # walk the loop in order, then shoelace area in YZ
    start = next(iter(verts_in)); order = [start]; prev = None; cur = start
    for _ in range(len(verts_in) + 1):
        nxts = [n for n in adj[cur] if n != prev]
        if not nxts: break
        nxt = nxts[0]; order.append(nxt); prev = cur; cur = nxt
        if cur == start: break
    ys = np.array([bm.verts[i].co.y for i in order]); zs = np.array([bm.verts[i].co.z for i in order])
    area = 0.5 * abs(float(np.sum(ys * np.roll(zs, -1) - zs * np.roll(ys, -1))))
    if area > CAP_MAX_AREA_M2:
        n_big += 1; continue            # macro loop (foot opening) -> NEVER cap
    res = bmesh.ops.triangle_fill(bm, edges=edgelist, use_beauty=True)
    if res.get('faces'): n_capped += 1
    else: n_open += 1
bm.normal_update(); bm.to_mesh(master.data); bm.free(); master.data.update()
print('  ankle flattened at x=%.1fmm: %d -> %d verts  (capped %d tube ends, %d open ribbons left, %d big loops skipped=opening preserved)' % (
    ANKLE_FLAT_X*1000, before_v, len(master.data.vertices), n_capped, n_open, n_big))

# export STL
stl_path = os.path.join(project, 'shoe_export.stl')
bpy.ops.object.select_all(action='DESELECT')
master.select_set(True)
bpy.context.view_layer.objects.active = master
bpy.ops.wm.stl_export(filepath=stl_path, export_selected_objects=True)
size_mb = os.path.getsize(stl_path) / (1024*1024)
print('  exported STL: %s (%.1f MB)' % (stl_path, size_mb))

# cleanup: delete the export copies
bpy.ops.object.delete()
# purge orphan mesh data
for block in list(bpy.data.meshes):
    if block.users == 0:
        bpy.data.meshes.remove(block)

# re-hide boot shell, verify originals intact
bpy.data.objects['left boot cutout meters'].hide_set(True)
print('\n=== ORIGINAL OBJECTS (untouched) ===')
for o in bpy.data.objects:
    if o.name in ('Lattice_OUTER','Lattice_INNER','Rims','Tongue','left boot cutout meters'):
        t = 'CURVE' if o.type=='CURVE' else 'MESH'
        print('  %-28s %s (intact)' % (o.name, t))
print('\nSAFE EXPORT COMPLETE - live curves preserved in shoe.blend')
