"""SAFE export (non-destructive): duplicates the curve objects, converts the COPIES
to mesh, joins them, exports STL, then DELETES the copies. Original curves untouched.
Does NOT include the boot base mesh (that's just a reference surface, not part of the print)."""
import bpy, os

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

# === FLATTEN THE ANKLE (-X) END INTO A PERFECT PLANE ===
# The round rim/tongue beads at the ankle bulge to ~-11.3mm and are not flat,
# so the boot rocks when printed ankle-down on the build plate. Slice off
# everything -X of this plane and CAP the cut so the whole -X end is one
# solid flat face that sits flush on the plate.
# (Non-destructive: this only touches the joined EXPORT copy.)
import bmesh
ANKLE_FLAT_X = -0.0100   # -10.0mm = flush with the shoe body's -X extent
before_v = len(master.data.vertices)
bm = bmesh.new(); bm.from_mesh(master.data)
geom = bm.verts[:] + bm.edges[:] + bm.faces[:]
bmesh.ops.bisect_plane(bm, geom=geom, plane_co=(ANKLE_FLAT_X,0,0), plane_no=(1,0,0),
                       dist=1e-7, clear_outer=False, clear_inner=False)
# delete everything strictly -X of the cut plane
bmesh.ops.delete(bm, geom=[v for v in bm.verts if v.co.x < ANKLE_FLAT_X - 1e-6], context='VERTS')
# cap the cut: fill boundary edges lying exactly on the plane -> solid flat face
bm.edges.ensure_lookup_table()
cut_edges = [e for e in bm.edges
             if len(e.link_faces) == 1
             and abs(e.verts[0].co.x - ANKLE_FLAT_X) < 1e-5
             and abs(e.verts[1].co.x - ANKLE_FLAT_X) < 1e-5]
if cut_edges:
    bmesh.ops.triangle_fill(bm, edges=cut_edges, use_beauty=True)
bm.normal_update(); bm.to_mesh(master.data); bm.free(); master.data.update()
print('  ankle flattened at x=%.1fmm: %d -> %d verts (capped %d cut edges)' % (
    ANKLE_FLAT_X*1000, before_v, len(master.data.vertices), len(cut_edges)))

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
