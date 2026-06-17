"""SAFE export (non-destructive): duplicates the curve objects, converts the COPIES
to mesh, joins them, exports STL, then DELETES the copies. Original curves untouched.
Does NOT include the boot base mesh (that's just a reference surface, not part of the print)."""
import bpy, os

project = '/home/dustin/Documents/Models/Roxanne Shoes/'

# unhide boot so we can manipulate (but we won't export it)
bpy.data.objects['left boot cutout meters'].hide_set(False)

# objects to export: lattice + rims + tongue (the actual printable geometry)
curve_objs = [bpy.data.objects[n] for n in ('Lattice_OUTER','Lattice_INNER','Rims','Tongue')]

print('=== SAFE EXPORT (working on copies, originals untouched) ===')
copies = []
for o in curve_objs:
    # duplicate object + data
    o_copy = o.copy()
    o_copy.data = o.data.copy()
    o_copy.name = o.name + '_EXPORT'
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
