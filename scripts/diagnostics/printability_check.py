import bpy

print('=== CURRENT SCENE OBJECTS ===')
total_eval_verts = 0
for o in bpy.data.objects:
    if o.type not in ('MESH','CURVE'): continue
    dg = bpy.context.evaluated_depsgraph_get()
    ev = o.evaluated_get(dg)
    if ev.type == 'MESH':
        em = ev.to_mesh()
        nv = len(em.vertices); nf = len(em.polygons)
        ev.to_mesh_clear()
    else:
        nv = nf = 0
    mods = [(m.name,m.type) for m in o.modifiers]
    print('  %-28s %-6s hide=%s  mods=%s  eval=%dv/%df' % (
        o.name, o.type, o.hide_viewport, mods, nv, nf))
    total_eval_verts += nv
print('\ntotal evaluated verts across all geometry: %d' % total_eval_verts)
