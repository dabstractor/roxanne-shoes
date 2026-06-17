import bpy, numpy as np, math

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

# BFS unwrap (identical to build_lattice.py)
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

# === TEST 1: unwrap continuity (tears). For each edge, |dphi_lift|; tears = outliers ===
dphi=[]; elen=[]
for e in mesh.edges:
    a,b=int(e.vertices[0]),int(e.vertices[1])
    dphi.append(abs(phi_lift[b]-phi_lift[a])); elen.append(np.linalg.norm(V[b]-V[a]))
dphi=np.array(dphi); elen=np.array(elen)+1e-9
print('=== UNWRAP CONTINUITY (per edge |dphi_lift|) ===')
print('median=%.4f  mean=%.4f  95pct=%.4f  MAX=%.4f' % (np.median(dphi), dphi.mean(), np.percentile(dphi,95), dphi.max()))
big=dphi[dphi>0.05]
print('edges with |dphi|>0.05 (level spacing 1/56=0.018): %d (%.3f%%)' % (len(big), 100*len(big)/len(dphi)))

# === TEST 2: robust gradient magnitude (edge-based, median) ===
inc=[[] for _ in range(len(V))]
for e in mesh.edges:
    a,b=int(e.vertices[0]),int(e.vertices[1])
    g=abs(phi_lift[b]-phi_lift[a])/max(np.linalg.norm(V[b]-V[a]),1e-9)
    inc[a].append(g); inc[b].append(g)
gradmag=np.array([np.median(x) if x else 0.0 for x in inc])   # robust per-vertex |grad|

# map onto dorsal top, find where grad is HIGH (= where lines bunch)
dorsal=np.array([v.index for v in mesh.vertices if v.co.z>0.005])
print('\n=== GRADIENT |grad phi_lift| on dorsal top (bunching = high) ===')
def rep(xlo,xhi,ylo,yhi,label):
    sel=[i for i in dorsal if xlo<=V[i,0]*1000<xhi and ylo<=V[i,1]*1000<yhi]
    if len(sel)<5: print('  %-16s n=%d'%('' if not sel else label,len(sel))); return
    g=gradmag[sel]
    print('  %-16s n=%4d  grad min=%.2f median=%.2f mean=%.2f MAX=%.2f' % (label,len(sel),g.min(),np.median(g),g.mean(),g.max()))
for xlo,xhi in [(65,75),(85,95),(105,112)]:
    print('  -- x %d-%dmm --' % (xlo,xhi))
    rep(xlo,xhi,-2.5,-0.5,'SEAM (y~-1.5)')
    rep(xlo,xhi,-8,-3,'side (y<-3)')
    rep(xlo,xhi,1,8,'top (+y)')

# === locate the actual gradient MAXIMUM on dorsal ===
dg=gradmag[dorsal]
order=dorsal[np.argsort(-dg)]
print('\n=== top 12 highest-gradient dorsal verts (bunching epicenter) ===')
for i in order[:12]:
    print('  x=%.1fmm y=%.1fmm z=%.1fmm  |grad|=%.2f' % (V[i,0]*1000,V[i,1]*1000,V[i,2]*1000,gradmag[i]))
