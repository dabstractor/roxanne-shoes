import bpy, numpy as np, math

boot=bpy.data.objects['left boot cutout meters']
mesh=boot.data
V=np.array([tuple(v.co) for v in mesh.vertices],dtype=float)

# For each vertex, compute geodesic-ish distance from the top ridge measured ALONG the surface
# (1-ring edge-path). Vertices far (in surface distance) from the top ridge get large 'u'.
# Lines of constant u then space by surface area, not by angle -> uniform on the flat top.

# top ridge verts: z near the per-slice max
xmin=float(V[:,0].min()); xmax=float(V[:,0].max())
NB=60; bed=np.linspace(xmin,xmax,NB+1)
bi=np.clip(np.digitize(V[:,0],bed)-1,0,NB-1)
ridge=np.zeros(len(V),dtype=bool)
for b in range(NB):
    members=[i for i in range(len(V)) if bi[i]==b]
    if not members: continue
    zmax=max(V[i,2] for i in members)
    for i in members:
        if V[i,2] > zmax-0.0008: ridge[i]=True
print('ridge seed verts:', ridge.sum())

# Dijkstra surface distance from ridge
from heapq import heappush, heappop
adj=[[] for _ in range(len(V))]
for e in mesh.edges:
    a,b=int(e.vertices[0]),int(e.vertices[1]); L=np.linalg.norm(V[a]-V[b])
    adj[a].append((b,L)); adj[b].append((a,L))
INF=1e9
dist=np.full(len(V),INF)
hq=[]
for i in range(len(V)):
    if ridge[i]: dist[i]=0.0; heappush(hq,(0.0,i))
while hq:
    d,c=heappop(hq)
    if d>dist[c]: continue
    for nb,L in adj[c]:
        nd=d+L
        if nd<dist[nb]: dist[nb]=nd; heappush(hq,(nd,nb))
dmax=dist[dist<INF].max()
print('surface distance from ridge: max %.1fmm' % (dmax*1000))

# u = surface distance / max (0 at ridge -> 1 at far edge). Check gradient uniformity.
u=dist/dmax
# dorsal top vertices: report u distribution
dorsal=np.array([v.index for v in mesh.vertices if v.co.z>0.005])
# for x columns, how spread is u? (uniform spacing => no bunching)
print('\n=== u (surface dist from ridge) uniformity on dorsal ===')
print('if u is well-spread per column, lines of constant-u will space evenly (no seam)')
for xlo in range(62,115,10):
    xhi=xlo+10
    col=[i for i in dorsal if xlo<=V[i,0]*1000<xhi]
    if len(col)<20: continue
    uu=u[col]
    # ideal: u uniformly fills its range. bunching = many verts at same u.
    # bin into 20, report max bin fraction
    hist,_=np.histogram(uu, bins=20, range=(uu.min(),uu.max()))
    print('  x %d-%dmm: n=%4d  u range %.2f-%.2f  maxbin=%.0f%%  (low=no bunch)' % (
        xlo,xhi,len(col),uu.min(),uu.max(),100*hist.max()/len(col)))

# save u to a vertex group so the lattice can use it
import bpy as _bpy
vg=boot.vertex_groups.get('surf_u')
if vg is None: vg=boot.vertex_groups.new(name='surf_u')
else:
    for i in range(len(V)): vg.remove([i])
for i in range(len(V)):
    vg.add([i], float(u[i]), 'REPLACE')
print('\nsaved surface-distance coordinate u to vertex group "surf_u"')
