"""Export the shoe WITHOUT the 4 latch posts (for re-import into Fusion as the base
geometry for a new latch design). Same non-destructive technique as safe_export_stl.py:
duplicates the curve/mesh objects, converts the COPIES to mesh, joins, exports STL,
then deletes the copies. Original curves untouched. Boot base mesh is NOT included
(reference surface only).

Posts excluded (the old latch system being replaced):
    Cuff_Upper_L, Cuff_Upper_R, Foot_Lower_L, Foot_Lower_R
"""
import bpy, os

project = '/home/dustin/Documents/Models/Roxanne Shoes/'

# unhide boot so we can manipulate (but we won't export it)
bpy.data.objects['left boot cutout meters'].hide_set(False)

# objects to export: lattice + V-band + ankle rim ONLY (NO POSTS)
EXPORT_NAMES = ('Lattice_OUTER', 'Lattice_INNER', 'V_Band', 'Ankle_Rim')
curve_objs = [bpy.data.objects[n] for n in EXPORT_NAMES]

print('=== NO-POSTS EXPORT (working on copies, originals untouched) ===')
copies = []
for o in curve_objs:
    o_copy = o.copy()
    o_copy.data = o.data.copy()
    o_copy.name = o.name + '_EXPORT'
    # o.copy() carries the object's modifier stack (e.g. Ankle_Rim's Solidify);
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

# export STL
stl_path = os.path.join(project, 'shoe_no_posts.stl')
bpy.ops.object.select_all(action='DESELECT')
master.select_set(True)
bpy.context.view_layer.objects.active = master
bpy.ops.wm.stl_export(filepath=stl_path, export_selected_objects=True)
size_mb = os.path.getsize(stl_path) / (1024 * 1024)
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
    if o.name in ('Lattice_OUTER', 'Lattice_INNER', 'V_Band', 'Ankle_Rim',
                  'Cuff_Upper_L', 'Cuff_Upper_R', 'Foot_Lower_L', 'Foot_Lower_R',
                  'left boot cutout meters'):
        t = 'CURVE' if o.type == 'CURVE' else 'MESH'
        print('  %-28s %s (intact)' % (o.name, t))
print('\nNO-POSTS EXPORT COMPLETE - live curves preserved in shoe.blend')
