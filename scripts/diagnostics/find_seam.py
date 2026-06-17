import bpy, numpy as np, math

# 1. list all objects + hidden state
print('=== OBJECTS ===')
for o in bpy.data.objects:
    print('  %-22s %-6s hidden_viewport=%s' % (o.name, o.type, o.hide_viewport))

# 2. reconstruct the phi seam location
boot=bpy.data.objects['left boot cutout meters']
mesh=boot.data
V=np.array([tuple(v.co) for v in mesh.vertices],dtype=float)
xmin=float(V[:,0].min()); xmax=float(V[:,0].max())
NB=60
bed=np.linspace(xmin,xmax,NB+1)
bi=np.clip(np.digitize(V[:,0],bed)-1,0,NB-1)
cy=np.zeros(NB); cz=np.zeros(NB); cnt=np.zeros(NB)
for i in range(len(V)):
    cy[bi[i]]+=V[i,1]; cz[bi[i]]+=V[i,2]; cnt[bi[i]]+=1
cnt[cnt==0]=1; cy/=cnt; cz/=cnt
def smooth1d(arr,passes,hw):
    a=arr.copy()
    for _ in range(passes):
        s=a.copy()
        for i in range(len(a)):
            lo=max(0,i-hw); hi=min(len(a)-1,i+hw); s[i]=a[lo:hi+1].mean()
        a=s
    return a
cy=smooth1d(cy,25,4)
cy_v=cy[bi]; cz_v=cz[bi]
dy=V[:,1]-cy_v; dz=V[:,2]-cz_v
gamma=np.arctan2(dy,-dz)
# seam = where gamma wraps: |gamma| near pi  => top center
seam_mask = np.abs(np.abs(gamma)-math.pi) < 0.25
print()
print('=== TOP SEAM location (phi wrap, gamma ~ +/-pi) ===')
sm=V[seam_mask]
if len(sm):
    print('seam verts: %d, X %.1f..%.1fmm, Y %.1f..%.1fmm, Z %.1f..%.1fmm' % (
        len(sm), sm[:,0].min()*1000, sm[:,0].max()*1000,
        sm[:,1].min()*1000, sm[:,1].max()*1000, sm[:,2].min()*1000, sm[:,2].max()*1000))

# 3. do lattice curve points pile up near the seam in the TOE-TOP region (x>78mm, z high)?
def pts(obj):
    out=[]
    for sp in obj.data.splines:
        for p in sp.points: out.append((p.co[0],p.co[1],p.co[2]))
    return np.array(out,dtype=float)

for nm in ('Lattice_OUTER','Lattice_INNER','Rims'):
    o=bpy.data.objects.get(nm)
    if o is None:
        print('%-14s NOT FOUND' % nm); continue
    P=pts(o)
    # toe-top region: x>0.078, z>0.015
    toe_top = P[(P[:,0]>0.078)&(P[:,2]>0.015)]
    # near seam = |y - cy(x)| small. approximate cy~0 in toe
    near_seam = toe_top[np.abs(toe_top[:,1])<0.002]
    print()
    print('=== %s ===' % nm)
    print('  total pts: %d' % len(P))
    print('  toe-top pts (x>78mm,z>15mm): %d' % len(toe_top))
    print('  of those, near top-seam (|y|<2mm): %d  (%.1f%%)' % (
        len(near_seam), 100*len(near_seam)/max(1,len(toe_top))))
    if len(toe_top):
        # histogram of |y| to see pile-up at y=0
        ay=np.abs(toe_top[:,1])*1000
        print('  toe-top |y| mm: min %.2f  median %.2f  max %.2f' % (ay.min(), np.median(ay), ay.max()))
        # count in bins
        bins=[0,1,2,3,5,8,12,20]
        for i in range(len(bins)-1):
            c=((ay>=bins[i])&(ay<bins[i+1])).sum()
            print('    |y| %2d-%2dmm: %d' % (bins[i],bins[i+1],c))
