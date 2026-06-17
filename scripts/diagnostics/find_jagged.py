import bpy, bmesh, numpy as np
boot = bpy.data.objects['left boot cutout meters']
rim = bpy.data.objects.get('Rims')
print('=== RIM SPINE CHECK ===')
if rim is None:
    print('No Rims object found. Jaggedness is NOT the rim.')
else:
    # the rim was built by chaining open boundary edges. Check ordering zigzag.
    sp = rim.data.splines[0]
    pts = np.array([(p.co[0]*1000, p.co[1]*1000, p.co[2]*1000) for p in sp.points])
    print('rim spline 0 points:', len(pts))
    # measure segment lengths and direction reversals
    segs = np.diff(pts, axis=0)
    seglens = np.sqrt((segs**2).sum(axis=1))
    print('segment lengths: min %.3f max %.3f mean %.3f mm' % (seglens.min(), seglens.max(), seglens.mean()))
    # zigzag = consecutive segments with negative dot product (backtracking)
    dots = (segs[:-1]*segs[1:]).sum(axis=1)
    n1 = np.sqrt((segs[:-1]**2).sum(axis=1)); n2 = np.sqrt((segs[1:]**2).sum(axis=1))
    cos = dots/(n1*n2+1e-9)
    reversals = int((cos < 0.0).sum())
    sharp = int((cos < 0.5).sum())
    print('direction reversals (cos<0, backtracking):', reversals, 'of', len(cos))
    print('sharp turns (cos<0.5):', sharp)
    print('>>> %s' % ('RIM IS ZIGZAGGING -> jagged tube on clean cut' if reversals>10 else 'rim ordering looks ok'))

print()
print('=== SOLIDIFY SHELL CHECK near V ===')
sol = boot.modifiers.get('GradientSolidify')
if sol:
    print('GradientSolidify present. offset=%.2f use_rim=%s' % (sol.offset, sol.use_rim))
    dg = bpy.context.evaluated_depsgraph_get()
    ev = boot.evaluated_get(dg); em = ev.to_mesh()
    # find evaluated verts near the V boundary region (dorsal, x in V range)
    near=[]
    for v in em.vertices:
        if v.co.x > -0.012 and v.co.x < 0.08 and v.co.z > 0.0:
            near.append((v.co.x*1000, v.co.y*1000, v.co.z*1000))
    print('evaluated dorsal verts near V region:', len(near), '(of %d total)' % len(em.vertices))
    ev.to_mesh_clear()
