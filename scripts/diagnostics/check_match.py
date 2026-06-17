import bpy, numpy as np
from mathutils.kdtree import KDTree

# rebuild segs the same way the script does, but just measure endpoint matching
import bmesh, math
boot=bpy.data.objects['left boot cutout meters']
mesh=boot.data
V=np.array([tuple(v.co) for v in mesh.vertices],dtype=float)
N=np.array([tuple(v.normal) for v in mesh.vertices],dtype=float)
bm=bmesh.new(); bm.from_mesh(mesh)
bmesh.ops.triangulate(bm, faces=list(bm.faces))
tris=np.array([[v.index for v in f.verts] for f in bm.faces],dtype=int)
bm.free()
# recompute phi_lift
xmin=float(V[:,0].min()); xmax=float(V[:,0].max())
NB=60; bed=np.linspace(xmin,xmax,NB+1)
bi=np.clip(np.digitize(V[:,0],bed)-1,0,NB-1)
cy=np.zeros(NB); cz=np.zeros(NB); cnt=np.zeros(NB)
for i in range(len(V)):
    cy[bi[i]]+=V[i,1]; cz[bi[i]]+=V[i,2]; cnt[bi[i]]+=1
cnt[cnt==0]=1; cy/=cnt; cz/=cnt
def sm(a,p,h):
    a=a.copy()
    for _ in range(p):
        s=a.copy()
        for i in range(len(a)):
            lo=max(0,i-h); hi=min(len(a)-1,i+hw if False else i+h); s[i]=a[lo:hi+1].mean()
        a=s
    return a
cy=sm(cy,25,4); cz=sm(cz,25,4)
cy_v=cy[bi]; cz_v=cz[bi]
gamma=np.arctan2(V[:,1]-cy_v, V[:,2]-cz_v)
phi=(gamma+math.pi)/(2*math.pi)  # approx, just for unwrap test
adj=[[] for _ in range(len(V))]
for e in mesh.edges:
    a,b=int(e.vertices[0]),int(e.vertices[1]); adj[a].append(b); adj[b].append(a)
phi_lift=np.zeros(len(V)); seen=np.zeros(len(V),dtype=bool); phi_lift[0]=phi[0]; seen[0]=True
from collections import deque
q=deque([0])
while q:
    c=q.popleft()
    for nb in adj[c]:
        if seen[nb]: continue
        seen[nb]=True; d=phi[nb]-phi[c]; d=d-round(d); phi_lift[nb]=phi_lift[c]+d; q.append(nb)
s_norm=(V[:,0]-xmin)/(xmax-xmin)
P=56; T=28; sign=1; offset=0.000868
def inorm(i,j,t):
    n=N[i]*(1-t)+N[j]*t; L=math.sqrt(n[0]**2+n[1]**2+n[2]**2) or 1; return n/L
segs=[]
for ti in range(len(tris)):
    ia,ib,ic=tris[ti]
    fa=P*phi_lift[ia]+sign*T*s_norm[ia]; fb=P*phi_lift[ib]+sign*T*s_norm[ib]; fc=P*phi_lift[ic]+sign*T*s_norm[ic]
    fmn=min(fa,fb,fc); fmx=max(fa,fb,fc); klo=math.ceil(fmn-1e-6); khi=math.floor(fmx+1e-6)
    if klo>khi: continue
    for L in range(klo,khi+1):
        pts=[]
        for (i,j,fi,fj) in ((ia,ib,fa,fb),(ib,ic,fb,fc),(ic,ia,fc,fa)):
            if fi!=fj and (fi-L)*(fj-L)<0:
                t=(L-fi)/(fj-fi); pts.append(V[i]*(1-t)+V[j]*t+inorm(i,j,t)*offset)
            elif fi==L:
                pts.append(V[i]+N[i]*offset)
        if len(pts)>=2: segs.append((pts[0],pts[1]))
print('segments:', len(segs))
# collect all endpoints, find nearest other-segment endpoint
allp=[]; owner=[]
for si,(a,b) in enumerate(segs):
    allp.append(a); owner.append(si); allp.append(b); owner.append(si)
allp=np.array(allp)
kd=KDTree(len(allp))
for i,p in enumerate(allp): kd.insert(p,i)
kd.balance()
nn=[]
for i,p in enumerate(allp):
    res=kd.find_n(p,4)
    for (co,idx,d) in res:
        if owner[idx]!=owner[i]:
            nn.append(d*1e6); break
nn=np.array(nn)
print('nearest OTHER-segment endpoint distance (microns):')
print('  min %.2f  median %.2f  mean %.2f  max %.2f' % (nn.min(), np.median(nn), nn.mean(), nn.max()))
for t in [1,5,10,15,20,50,100]:
    print('  within %d um: %d (%.0f%%)' % (t, int((nn<=t).sum()), 100*(nn<=t).mean()))
