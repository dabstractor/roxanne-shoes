"""Export pipeline (Path B): convert curves to mesh, apply modifiers, join, export STL.
Keeps the original .blend with live curves intact."""
import bpy, bmesh, os

project = '/home/dustin/Documents/Models/Roxanne Shoes/'

# 1. Convert each CURVE object to mesh
curve_objs = [o for o in bpy.data.objects if o.type == 'CURVE']
print('=== STEP 1: Convert curves to mesh ===')
for o in curve_objs:
    # make a backup copy of the curve data first (so we can restore if needed)
    bpy.context.view_layer.objects.active = o
    o.select_set(True)
    bpy.ops.object.convert(target='MESH')
    print('  converted %s -> MESH (%d verts)' % (o.name, len(o.data.vertices)))
    o.select_set(False)

# 2. Apply the GradientSolidify modifier on the boot shell
print('\n=== STEP 2: Apply Solidify modifier ===')
boot = bpy.data.objects['left boot cutout meters']
boot.hide_set(False)
bpy.context.view_layer.objects.active = boot
boot.select_set(True)
for mod in list(boot.modifiers):
    bpy.ops.object.modifier_apply(modifier=mod.name)
    print('  applied %s' % mod.name)
boot.select_set(False)

# 3. Join all geometry objects into one
print('\n=== STEP 3: Join all objects ===')
geom_objs = [o for o in bpy.data.objects if o.type == 'MESH' and o.name != 'left boot cutout BACKUP']
# make the boot shell the active (master) object
bpy.context.view_layer.objects.active = boot
for o in geom_objs:
    o.select_set(True)
bpy.ops.object.join()
print('  joined into: %s (%d verts, %d faces)' % (boot.name, len(boot.data.vertices), len(boot.data.polygons)))

# 4. Export STL
print('\n=== STEP 4: Export STL ===')
stl_path = os.path.join(project, 'shoe_export.stl')
# select only the joined object
for o in bpy.data.objects: o.select_set(False)
boot.select_set(True)
bpy.context.view_layer.objects.active = boot
bpy.ops.wm.stl_export(filepath=stl_path, export_selected_objects=True)
size_mb = os.path.getsize(stl_path) / (1024*1024)
print('  exported: %s (%.1f MB)' % (stl_path, size_mb))

print('\n=== DONE ===')
print('STL: %s' % stl_path)
print('verts: %d, faces: %d' % (len(boot.data.vertices), len(boot.data.polygons)))
print('NOTE: original .blend still has live curves/modifiers in its pre-join state')
print('      (this script mutated the working file; re-run build scripts to restore)')
