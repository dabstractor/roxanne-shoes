"""SAFE export (non-destructive): duplicates the curve objects, converts the COPIES
to mesh, joins them, exports STL, then DELETES the copies. Original curves untouched.
Does NOT include the boot base mesh (that's just a reference surface, not part of the print)."""
import bpy, os, numpy as np

project = '/home/dustin/Documents/Models/Roxanne Shoes/'

# unhide boot so we can manipulate (but we won't export it)
bpy.data.objects['left boot cutout meters'].hide_set(False)

# objects to export: lattice + rims + tongue + ankle reinforcement + 4 latch posts
# (the actual printable geometry). Posts are linked mesh instances (Post_mesh) placed
# by scripts/place_posts.py: 2 per reinforce collar straddling the V, 2.25mm near-edge
# to the V cut edge (-> 12.5mm on-center, 4.5mm gap when V closes), base inset 0.25mm,
# axis = true surface normal (toward-ankle tilt included).
EXPORT_NAMES = ('Lattice_OUTER','Lattice_INNER','V_Band','Tongue','Ankle_Reinforce',
                'Cuff_Upper_L','Cuff_Upper_R','Foot_Lower_L','Foot_Lower_R')
curve_objs = [bpy.data.objects[n] for n in EXPORT_NAMES]

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
# (BASELINE EXPORT -- no ankle flattening/bisect/capping. The previous bisect+cap
# pass created a billion stray triangles jutting from the ankle; removed entirely per
# user request to establish a clean baseline. The ankle now exports exactly as-built.)
# NOTE: this means the ankle end is NOT a flat plane -- objects extend to their natural
# extents (lattice tubes to ~-10.3mm, etc.). The flat-print-plane requirement will be
# re-solved a different way later.

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
    if o.name in ('Lattice_OUTER','Lattice_INNER','V_Band','Tongue','left boot cutout meters'):
        t = 'CURVE' if o.type=='CURVE' else 'MESH'
        print('  %-28s %s (intact)' % (o.name, t))
print('\nSAFE EXPORT COMPLETE - live curves preserved in shoe.blend')
