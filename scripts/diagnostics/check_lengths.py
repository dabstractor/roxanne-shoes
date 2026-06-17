import bpy, numpy as np
for nm in ('Lattice_OUTER','Lattice_INNER'):
    o=bpy.data.objects.get(nm)
    if not o: continue
    lengths=[len(sp.points) for sp in o.data.splines]
    lengths=np.array(lengths)
    print('=== %s ===' % nm)
    print('total splines: %d' % len(lengths))
    print('pts/spline: min %d  median %d  mean %.0f  max %d' % (lengths.min(), np.median(lengths), lengths.mean(), lengths.max()))
    print('splines with <10 pts (fragments): %d (%.0f%%)' % ((lengths<10).sum(), 100*(lengths<10).mean()))
    print('splines with >200 pts (long lines): %d (%.0f%%)' % ((lengths>200).sum(), 100*(lengths>200).mean()))
    # total points
    print('total points: %d' % lengths.sum())
