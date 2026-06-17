"""Cut away the solid shell in the vented zone so only the lattice remains there.
Keeps the solid shell only at the toe block (high gradient weight).
Run BEFORE export_stl.py, AFTER build_lattice.py has set up the gradient."""
import bpy, bmesh, numpy as np

boot = bpy.data.objects['left boot cutout meters']
mesh = boot.data
boot.hide_set(False)
if bpy.context.object and bpy.context.object.mode != 'OBJECT':
    bpy.ops.object.mode_set(mode='OBJECT')
for o in bpy.context.view_layer.objects: o.select_set(False)
bpy.context.view_layer.objects.active = boot; boot.select_set(True)

# --- 1. Apply GradientSolidify so we have the actual thickened geometry ---
for mod in list(boot.modifiers):
    if mod.type == 'SOLIDIFY':
        bpy.ops.object.modifier_apply(modifier=mod.name)
        print('applied %s' % mod.name)

# --- 2. Read weights from mesh data (BEFORE bmesh, since BMVert has no .groups) ---
vg = boot.vertex_groups.get('gradient')
vgi = vg.index if vg else None
print('gradient group:', 'found' if vg else 'MISSING')

# build weight lookup array indexed by mesh vertex index
vert_weights = np.zeros(len(mesh.vertices))
if vgi is not None:
    for v in mesh.vertices:
        for g in v.groups:
            if g.group == vgi:
                vert_weights[v.index] = g.weight; break

bm = bmesh.new(); bm.from_mesh(mesh)
bm.faces.ensure_lookup_table()
bm.verts.ensure_lookup_table()

# --- 3. Delete faces in the VENTED zone (low weight) ---
VENT_THRESHOLD = 0.40   # keep shell where weight > 0.40 (toe block), delete elsewhere

to_delete = []
to_keep = []
for f in bm.faces:
    # average weight of face verts via mesh vertex index
    weights = [vert_weights[v.index] for v in f.verts]
    avg_w = sum(weights)/len(weights) if weights else 0.0
    if avg_w < VENT_THRESHOLD:
        to_delete.append(f)
    else:
        to_keep.append(f)

print('shell faces: %d total, %d to DELETE (vented), %d to KEEP (toe block)' % (
    len(bm.faces), len(to_delete), len(to_keep)))

# report the toe block extent
if to_keep:
    xs = [f.calc_center_median().x*1000 for f in to_keep]
    print('toe block retained: x %.0f..%.0fmm' % (min(xs), max(xs)))

bmesh.ops.delete(bm, geom=to_delete, context='FACES')
# remove loose verts
loose = [v for v in bm.verts if not v.link_faces]
if loose:
    bmesh.ops.delete(bm, geom=loose, context='VERTS')
    print('removed %d loose verts' % len(loose))
bm.to_mesh(mesh); bm.free(); mesh.update()
print('shell after cut: %d verts, %d faces' % (len(mesh.vertices), len(mesh.polygons)))
print('=== VENTED ZONE SHELL REMOVED - lattice is now the breathable surface ===')
