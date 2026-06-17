import bpy, numpy as np, math

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
cy=smooth1d(cy,25,4); cz=smooth1d(cz,25,4)
cx_bed=(bed[:-1]+bed[1:])/2.0
def cy_at(x):
    if x<=cx_bed[0]: return cy[0]
    if x>=cx_bed[-1]: return cy[-1]
    for i in range(NB-1):
        if cx_bed[i]<=x<=cx_bed[i+1]:
            t=(x-cx_bed[i])/(cx_bed[i+1]-cx_bed[i]); return cy[i]+(cy[i+1]-cy[i])*t
    return 0.0

def endpoints(obj):
    eps=[]
    for sp in obj.data.splines:
        pts=sp.points
        if len(pts)>=2:
            eps.append((pts[0].co[0],pts[0].co[1],pts[0].co[2]))
            eps.append((pts[-1].co[0],pts[-1].co[1],pts[-1].co[2]))
    return np.array(eps,dtype=float)

for nm in ('Lattice_OUTER','Lattice_INNER'):
    o=bpy.data.objects.get(nm)
    if not o: continue
    ep=endpoints(o)
    # region: mid-toe, NO V here (x between 80 and 110mm), away from toe-tip singularity
    mid=ep[(ep[:,0]>0.080)&(ep[:,0]<0.110)]
    if len(mid)==0:
        print('%s: no endpoints in mid-toe region' % nm); continue
    # distance of each endpoint from the apex centerline (in y)
    dy=np.array([abs(p[1]-cy_at(p[0])) for p in mid])*1000
    apex_cluster=int((dy<0.3).sum())
    print('%s: mid-toe endpoints=%d  clustered at apex(|dy|<0.3mm)=%d  median|dy|=%.2fmm' % (
        nm, len(mid), apex_cluster, np.median(dy)))
    print('   -> %s' % ('CLEAN: lines cross apex continuously' if apex_cluster==0 else 'STILL BREAKING at apex'))
