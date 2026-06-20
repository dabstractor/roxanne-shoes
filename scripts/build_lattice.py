import bpy, bmesh, math, numpy as np
from collections import defaultdict, deque
from mathutils.kdtree import KDTree

boot = bpy.data.objects['left boot cutout meters']
mesh = boot.data
for nm in ('Lattice_OUTER','Lattice_INNER','Lattice','Bridges'):
    old=bpy.data.objects.get(nm)
    if old: bpy.data.objects.remove(old, do_unlink=True)
if bpy.context.object and bpy.context.object.mode != 'OBJECT':
    bpy.ops.object.mode_set(mode='OBJECT')

# --- RESTORE UNCUT MESH (apex intact = clean unwrap = no tear) ---
bak = bpy.data.objects.get('left boot cutout BACKUP')
if bak is not None:
    boot.data = bak.data.copy()
    mesh = boot.data
    print('RESTORED uncut mesh:', len(mesh.vertices), 'verts')

V = np.array([tuple(v.co) for v in mesh.vertices], dtype=float)
N = np.array([tuple(v.normal) for v in mesh.vertices], dtype=float)
bm=bmesh.new(); bm.from_mesh(mesh)
bmesh.ops.triangulate(bm, faces=list(bm.faces))
tris=np.array([[v.index for v in f.verts] for f in bm.faces], dtype=int)
bm.free()

# recompute gradient on the (uncut) mesh so weights match current topology
xs0=[v.co.x for v in mesh.vertices]; zs0=[v.co.z for v in mesh.vertices]
xmin0,xmax0=min(xs0),max(xs0); zmin0,zmax0=min(zs0),max(zs0)
def _ss0(a,b,x):
    t=max(0.0,min(1.0,(x-a)/(b-a) if b!=a else 0.0)); return t*t*(3-2*t)
vg=boot.vertex_groups.get('gradient')
if vg is None: vg=boot.vertex_groups.new(name='gradient')
vgi=vg.index
for v in mesh.vertices:
    fx=(v.co.x-xmin0)/(xmax0-xmin0) if xmax0!=xmin0 else 0
    fz=(v.co.z-zmin0)/(zmax0-zmin0) if zmax0!=zmin0 else 0
    base=_ss0(0.45,0.92,fx); dorsal=0.3+0.7*_ss0(0.30,0.90,fz)
    vg.add([v.index], max(0.0,min(1.0,0.15+(1-0.15)*base*dorsal)), 'REPLACE')
w=np.zeros(len(V))
for v in mesh.vertices:
    for g in v.groups:
        if g.group==vgi: w[v.index]=g.weight; break

# centerline (smoothed, identical params)
xmin=float(V[:,0].min()); xmax=float(V[:,0].max())
NB=60
bed=np.linspace(xmin,xmax,NB+1)
bi=np.clip(np.digitize(V[:,0],bed)-1,0,NB-1)
cy=np.zeros(NB); cz=np.zeros(NB); cnt=np.zeros(NB)
for i in range(len(V)):
    cy[bi[i]]+=V[i,1]; cz[bi[i]]+=V[i,2]; cnt[bi[i]]+=1
cnt[cnt==0]=1; cy/=cnt; cz/=cnt
def smooth1d(arr, passes, hw):
    a=arr.copy()
    for _ in range(passes):
        s=a.copy()
        for i in range(len(a)):
            lo=max(0,i-hw); hi=min(len(a)-1,i+hw); s[i]=a[lo:hi+1].mean()
        a=s
    return a
cy=smooth1d(cy,25,4); cz=smooth1d(cz,25,4)
cx_bed=(bed[:-1]+bed[1:])/2.0
cy_v=cy[bi]; cz_v=cz[bi]
dy=V[:,1]-cy_v; dz=V[:,2]-cz_v
gamma=np.arctan2(dy, -dz)
phi=(math.pi - gamma)/(2.0*math.pi)   # 0..1, wraps at top apex
s_norm=(V[:,0]-xmin)/(xmax-xmin) if xmax!=xmin else np.zeros(len(V))

# ===== GLOBAL UNWRAP of phi (lift so apex is continuous, not a jump) =====
adj=[[] for _ in range(len(V))]
for e in mesh.edges:
    a,b=int(e.vertices[0]),int(e.vertices[1])
    adj[a].append(b); adj[b].append(a)
phi_lift=np.zeros(len(V))
seen=np.zeros(len(V),dtype=bool)
phi_lift[0]=phi[0]; seen[0]=True
q=deque([0])
nseen=1
while q:
    c=q.popleft()
    for nb in adj[c]:
        if seen[nb]: continue
        seen[nb]=True; nseen+=1
        d=phi[nb]-phi[c]
        d=d - round(d)   # bring diff into [-0.5,0.5] => unwrap across seam
        phi_lift[nb]=phi_lift[c]+d
        q.append(nb)
print('global unwrap: %d/%d verts visited' % (nseen, len(V)))
print('phi_lift range: %.2f .. %.2f  (span ~ %.2f wraps)' % (phi_lift.min(), phi_lift.max(), phi_lift.max()-phi_lift.min()))

# === TEAR CHECK: count edges with full-unit jumps (these cause the seam artifacts) ===
_n_tears=0
for e in mesh.edges:
    _a,_b=int(e.vertices[0]),int(e.vertices[1])
    if abs(phi_lift[_b]-phi_lift[_a]) > 0.5: _n_tears+=1
print('TEAR CHECK: edges with >0.5 jump:', _n_tears, '(MUST be 0 on uncut mesh; was 309 on cut mesh)')

# KNOBS
N_LINES=56; SWEEP=0.50; RIB_RADIUS=0.000459; OFFSET_OUT=0.000448; OFFSET_IN=0.000448
# RIB_RADIUS 0.51->0.459mm (-10%) for harder TPU. OFFSET unchanged (sole-touch guarantee).
SMOOTH_ITERS=6; TOE_SCALE=2.27
# TOE_SCALE 2.55->2.27 so the toe (thickest part of the gradient) drops ~20% total
# (0.51*2.55=1.30mm -> 0.459*2.27=1.04mm), 10% beyond the base reduction.
P=N_LINES; T=SWEEP*N_LINES

def interp_normal(i,j,t):
    n=N[i]*(1-t)+N[j]*t; ln=math.sqrt(n[0]**2+n[1]**2+n[2]**2) or 1.0
    return n/ln

def iso_family(sign, offset):
    segs=[]
    for ti in range(len(tris)):
        ia,ib,ic=tris[ti]
        # PER-TRIANGLE LOCAL UNWRAP: the global phi_lift has an unavoidable branch cut
        # (~P jump). Locally bring pb,pc within P/2 of pa so contours cross cleanly.
        pa=P*phi_lift[ia]; pb=P*phi_lift[ib]; pc=P*phi_lift[ic]
        pb=pa+(pb-pa-round((pb-pa)/P)*P)
        pc=pa+(pc-pa-round((pc-pa)/P)*P)
        sa=sign*T*s_norm[ia]; sb=sign*T*s_norm[ib]; sc=sign*T*s_norm[ic]
        fa=pa+sa; fb=pb+sb; fc=pc+sc
        fmn=min(fa,fb,fc); fmx=max(fa,fb,fc)
        klo=math.ceil(fmn-1e-6); khi=math.floor(fmx+1e-6)
        if klo>khi: continue
        for L in range(klo,khi+1):
            pts=[]
            for (i,j,fi,fj) in ((ia,ib,fa,fb),(ib,ic,fb,fc),(ic,ia,fc,fa)):
                if fi!=fj and (fi-L)*(fj-L)<0:
                    t=(L-fi)/(fj-fi)
                    nrm=interp_normal(i,j,t)
                    pts.append(V[i]*(1-t)+V[j]*t + nrm*offset)
                elif fi==L:
                    pts.append(V[i] + N[i]*offset)
            if len(pts)>=2: segs.append((pts[0],pts[1]))
    return segs

def chain(segs, tol=6e-6):
    # TOLERANT chaining via KDTree: endpoints match within tol (distinct contours are ~20um apart,
    # true matches are <1um). Walks each line forward+backward, picking the nearest collinear neighbor.
    nseg=len(segs)
    pts=np.array([p for s in segs for p in s], dtype=float)   # [seg0a,seg0b,seg1a,seg1b,...]
    kd=KDTree(len(pts)); 
    for i,p in enumerate(pts): kd.insert(p,i)
    kd.balance()
    # global endpoint idx = seg*2 + end
    def gidx(seg,end): return seg*2+end
    seg_used=np.zeros(nseg, dtype=bool)
    loops=[]
    for s0 in range(nseg):
        if seg_used[s0]: continue
        seg_used[s0]=True
        a=pts[gidx(s0,0)].copy(); b=pts[gidx(s0,1)].copy()
        loop=[a,b]
        for direction in (1,-1):
            if direction==1:
                cur_g=gidx(s0,1); cur=b.copy(); prev=a.copy()
                append=lambda v: loop.append(v)
            else:
                cur_g=gidx(s0,0); cur=a.copy(); prev=b.copy()
                append=lambda v: loop.insert(0,v)
            guard=0
            while guard<30000:
                guard+=1
                res=kd.find_n(cur, 8)
                best=None; bestd=1e9
                for (co,idx,d) in res:
                    if d>tol: break
                    s,e=divmod(idx,2)
                    if seg_used[s]: continue
                    if d<bestd: bestd=d; best=(s,e)
                if best is None: break
                s,e=best; other=gidx(s,1-e); nxt=pts[other]
                seg_used[s]=True
                append(nxt.copy()); prev=cur.copy(); cur=nxt.copy()
        if len(loop)>=2: loops.append(loop)
    return loops

def smooth_loop(pts, iters):
    if len(pts)<3: return pts
    pts=[p.copy() for p in pts]
    for _ in range(iters):
        new=pts[:]
        for i in range(1,len(pts)-1):
            new[i]=(pts[i-1]+pts[i+1])*0.5
        pts=new
    return pts

kd_w=KDTree(len(V))
for i,c in enumerate(V): kd_w.insert(c,i)
kd_w.balance()

loops_out=[smooth_loop(p,SMOOTH_ITERS) for p in chain(iso_family(+1,+OFFSET_OUT))]
loops_in =[smooth_loop(p,SMOOTH_ITERS) for p in chain(iso_family(-1,-OFFSET_IN))]

def make_curve(name, loops):
    crv=bpy.data.curves.new(name,'CURVE')
    crv.dimensions='3D'; crv.bevel_depth=RIB_RADIUS; crv.bevel_resolution=2; crv.use_fill_caps=True; crv.resolution_u=12
    for pts in loops:
        sp=crv.splines.new('POLY'); sp.points.add(len(pts)-1)
        for i,p in enumerate(pts):
            co,idx,dist=kd_w.find((float(p[0]),float(p[1]),float(p[2])))
            wt=w[idx]; rad=1.0+(TOE_SCALE-1.0)*wt
            sp.points[i].co=(float(p[0]),float(p[1]),float(p[2]),1.0)
            sp.points[i].radius=rad
    o=bpy.data.objects.new(name,crv); bpy.context.collection.objects.link(o); o.parent=boot
    return o

make_curve('Lattice_OUTER', loops_out)
make_curve('Lattice_INNER', loops_in)
boot.hide_set(True)

# ===== MEASURE: are lines continuous across the apex? density uniform? =====
def cy_at(x):
    if x<=cx_bed[0]: return cy[0]
    if x>=cx_bed[-1]: return cy[-1]
    for i in range(NB-1):
        if cx_bed[i]<=x<=cx_bed[i+1]:
            t=(x-cx_bed[i])/(cx_bed[i+1]-cx_bed[i]); return cy[i]+(cy[i+1]-cy[i])*t
    return 0.0
def cz_at(x):
    if x<=cx_bed[0]: return cz[0]
    if x>=cx_bed[-1]: return cz[-1]
    for i in range(NB-1):
        if cx_bed[i]<=x<=cx_bed[i+1]:
            t=(x-cx_bed[i])/(cx_bed[i+1]-cx_bed[i]); return cz[i]+(cz[i+1]-cz[i])*t
    return 0.0
def ends(loops):
    e=[]
    for pts in loops:
        for ei in (0,-1):
            p=pts[ei]
            g=math.atan2(float(p[1])-cy_at(float(p[0])), -(float(p[2])-cz_at(float(p[0]))))
            e.append((float(p[0])*1000, abs(g)))
    return e

eo=ends(loops_out); ei=ends(loops_in)
# apex terminations = ends with |g| near pi AND not at a mesh boundary (x<-8 ankle or x>118 toe)
def apex_terms(ends):
    n=0
    for x,g in ends:
        if g>math.pi-0.3 and -8<x<118: n+=1
    return n
print('=== CONTINUITY CHECK ===')
print('OUTER loops=%d  ends=%d  apex-terminations(mid-surface)=%d' % (len(loops_out),len(eo),apex_terms(eo)))
print('INNER loops=%d  ends=%d  apex-terminations(mid-surface)=%d' % (len(loops_in),len(ei),apex_terms(ei)))

# apex density: % of toe-top points near apex centerline
def apex_density(name, loops):
    pts=[]
    for p in loops:
        pts+=p
    pts=np.array([(float(p[0]),float(p[1]),float(p[2])) for p in pts])
    tt=pts[(pts[:,0]>0.078)&(pts[:,2]>0.015)]
    if len(tt)==0: return
    near=sum(1 for p in tt if abs(p[1]-cy_at(p[0]))<0.001)
    print('%s toe-top pts=%d  near-apex(%%)=%.1f  median|y|=%.2fmm' % (name,len(tt),100*near/len(tt), np.median(np.abs(tt[:,1]-np.array([cy_at(p[0]) for p in tt])))*1000))
apex_density('OUTER',loops_out); apex_density('INNER',loops_in)
print('(apex-terminations should be ~0; near-apex%% should be low/uniform, NOT ~86%%)')
