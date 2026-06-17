import bpy, bmesh, numpy as np, math

boot=bpy.data.objects['left boot cutout meters']
mesh=boot.data
V=np.array([tuple(v.co) for v in mesh.vertices],dtype=float)
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
            lo=max(0,i-h); hi=min(len(a)-1,i+h); s[i]=a[lo:hi+1].mean()
        a=s
    return a
cy=sm(cy,25,4); cz=sm(cz,25,4)
cy_v=cy[bi]; cz_v=cz[bi]
dy=V[:,1]-cy_v; dz=V[:,2]-cz_v
gamma=np.arctan2(dy,-dz)
phi=(math.pi-gamma)/(2*math.pi)

# global unwrap
from collections import deque
adj=[[] for _ in range(len(V))]
for e in mesh.edges:
    a,b=int(e.vertices[0]),int(e.vertices[1]); adj[a].append(b); adj[b].append(a)
phi_lift=np.zeros(len(V)); seen=np.zeros(len(V),dtype=bool); phi_lift[0]=phi[0]; seen[0]=True
q=deque([0])
while q:
    c=q.popleft()
    for nb in adj[c]:
        if seen[nb]: continue
        seen[nb]=True; d=phi[nb]-phi[c]; d=d-round(d); phi_lift[nb]=phi_lift[c]+d; q.append(nb)

# surface gradient of phi_lift at each vertex (1-ring)
gradmag=np.zeros(len(V))
for v in mesh.vertices:
    i=v.index
    if len(adj[i])<2: continue
    # gradient via least-squares over neighbors projected to tangent plane
    nrm=np.array([v.normal.x,v.normal.y,v.normal.z])
    # build local 1-ring differences
    nb=adj[i]
    A=[]; b=[]
    p0=V[i]
    for j in nb:
        d=V[j]-p0; d=d-d*np.dot(d,nrm)*nrm  # project to tangent
        if np.dot(d,d)<1e-12: continue
        A.append([d[0],d[1],d[2]])
        b.append(phi_lift[j]-phi_lift[i])
    if len(A)<2: continue
    A=np.array(A); b=np.array(b)
    g,_,_,_=np.linalg.lstsq(A,b,rcond=None)
    gradmag[i]=np.linalg.norm(g)

# report grad magnitude on the dorsal surface, binned by (x, y)
print('=== FIELD GRADIENT |grad(phi_lift)| on dorsal top ===')
print('(SMALL gradient = lines BUNCH there = seam. LARGE = lines spread out.)')
# dorsal verts
dorsal=np.array([v.index for v in mesh.vertices if v.co.z>0.005])
print('dorsal verts:', len(dorsal))
# focus on the seam region x=62-112, and compare seam band (y~-1.5) vs off-seam (y<-3 or y>0)
def report(xlo,xhi,ylo,yhi,label):
    sel=[i for i in dorsal if xlo<=V[i,0]*1000<xhi and ylo<=V[i,1]*1000<yhi]
    if len(sel)<5:
        print('  %-20s n=%d (too few)' % (label,len(sel))); return
    g=gradmag[sel]
    print('  %-20s n=%4d  |grad| min=%.1f median=%.1f mean=%.1f max=%.1f' % (
        label,len(sel),g.min(),np.median(g),g.mean(),g.max()))
for xlo,xhi in [(62,70),(80,90),(100,110)]:
    print('  -- x %d-%dmm --' % (xlo,xhi))
    report(xlo,xhi,-2.5,-0.5,'SEAM band (y~-1.5)')
    report(xlo,xhi,-10,-3,'off-seam (far y)')
    report(xlo,xhi,0,10,'off-seam (+y)')
