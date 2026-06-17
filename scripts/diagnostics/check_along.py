import bpy, numpy as np
for nm in ('Lattice_OUTER','Lattice_INNER'):
    o=bpy.data.objects.get(nm)
    if not o: continue
    splines=list(o.data.splines)
    print('=== %s: %d splines ===' % (nm,len(splines)))
    # classify each: does it run ALONG the seam band (small y-range near -1.5, wide x-range)?
    along=[]; across=[]; other=[]
    for sp in splines:
        pts=np.array([(p.co[0],p.co[1],p.co[2]) for p in sp.points])
        if len(pts)<5: continue
        xr=(pts[:,0].max()-pts[:,0].min())*1000
        yr=(pts[:,1].max()-pts[:,1].min())*1000
        ymean=pts[:,1].mean()*1000
        # dorsal portion only
        dmask=pts[:,2]>0.005
        if dmask.sum()<5: 
            other.append(len(pts)); continue
        dpts=pts[dmask]
        dxr=(dpts[:,0].max()-dpts[:,0].min())*1000
        dyr=(dpts[:,1].max()-dpts[:,1].min())*1000
        dym=dpts[:,1].mean()*1000
        # "along seam": wide in x, narrow in y, near y=-1.5
        if dxr>15 and dyr<4 and -3.5<dym<0.5:
            along.append((dxr,dyr,dym,len(pts)))
        else:
            across.append((dxr,dyr,dym,len(pts)))
    print('  ALONG-seam lines (wide x, narrow y, near y=-1.5): %d' % len(along))
    for ax,ay,am,n in along[:6]:
        print('     xrange=%.0fmm yrange=%.0fmm ymean=%.1f npts=%d' % (ax,ay,am,n))
    print('  normal (across/other): %d' % len(across))
    # how many total points are in along-seam lines vs normal?
    ap=sum(n for _,_,_,n in along); np_=sum(n for _,_,_,n in across)
    print('  points in along-seam lines: %d (%.0f%%) | in normal lines: %d' % (ap, 100*ap/(ap+np_+1e-9), np_))
