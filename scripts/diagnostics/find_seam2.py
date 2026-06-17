import bpy, numpy as np, math

def all_pts(obj):
    out=[]
    for sp in obj.data.splines:
        for p in sp.points: out.append((p.co[0],p.co[1],p.co[2]))
    return np.array(out,dtype=float)

for nm in ('Lattice_OUTER','Lattice_INNER'):
    o=bpy.data.objects.get(nm)
    if not o: continue
    P=all_pts(o)
    # dorsal top surface: z>0 (and x in foot body, not the degenerate tip)
    top=P[(P[:,2]>0.005)&(P[:,0]>0.01)&(P[:,0]<0.115)]
    print('=== %s: %d dorsal pts ===' % (nm,len(top)))
    # bin by x (5mm columns), within each column find the densest y-band (0.5mm wide)
    xs=top[:,0]*1000; ys=top[:,1]*1000
    xbins=np.arange(10,115,5)
    print('  Xmm  densest_y(mm)  pts_in_densest_band  total_in_col  ratio')
    seam_ys=[]
    for xb in xbins:
        col=(xs>=xb)&(xs<xb+5)
        if col.sum()<20: continue
        cy=ys[col]
        # sliding 0.5mm window to find densest y
        best_y=None; best_n=0
        for yc in np.arange(cy.min(),cy.max(),0.3):
            n=((cy>=yc)&(cy<yc+0.5)).sum()
            if n>best_n: best_n=n; best_y=yc+0.25
        total=col.sum()
        ratio=best_n/total
        flag='  <-- DENSE BAND' if ratio>0.15 else ''
        print('  %3d   %+7.2f        %3d              %4d        %.2f%s' % (xb+2.5, best_y, best_n, total, ratio, flag))
        if ratio>0.15: seam_ys.append((xb+2.5, best_y))
    if seam_ys:
        ys_only=[y for _,y in seam_ys]
        print('  -> densest band y values: min %.2f max %.2f mean %.2f (consistent => a SEAM line)' % (min(ys_only),max(ys_only),sum(ys_only)/len(ys_only)))
    print()
