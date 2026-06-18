"""Tongue v4: thatched crosshatch lattice living on a single curved surface.
Conforms to dorsal at tip (firm attachment to ribbing), arcs toward centerline,
cross-section domed to match shoe curvature. Two diagonal families crossing
on the SAME surface (touch at intersections, single layer)."""
import bpy, bmesh, math, numpy as np
from mathutils.kdtree import KDTree

boot = bpy.data.objects['left boot cutout meters']
mesh = boot.data
for nm in ('Tongue',):
    old=bpy.data.objects.get(nm)
    if old: bpy.data.objects.remove(old, do_unlink=True)
# clean up any old hinge objects from previous runs
for o in list(bpy.data.objects):
    if o.name.startswith('Tongue_hinge'):
        bpy.data.objects.remove(o, do_unlink=True)

V=np.array([tuple(v.co) for v in mesh.vertices],dtype=float)
xmin=float(V[:,0].min()); xmax=float(V[:,0].max())
NB=60; bed=np.linspace(xmin,xmax,NB+1)
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
ridge=np.zeros(NB)
for b in range(NB):
    members=[i for i in range(len(V)) if bi[i]==b]
    ridge[b]=max((V[i,2] for i in members), default=cz[b])
ridge=smooth1d(ridge,15,3)
cx_bed=(bed[:-1]+bed[1:])/2.0
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
def ridge_at(x):
    if x<=cx_bed[0]: return ridge[0]
    if x>=cx_bed[-1]: return ridge[-1]
    for i in range(NB-1):
        if cx_bed[i]<=x<=cx_bed[i+1]:
            t=(x-cx_bed[i])/(cx_bed[i+1]-cx_bed[i]); return ridge[i]+(ridge[i+1]-ridge[i])*t
    return 0.0

kd=KDTree(len(V))
for i,c in enumerate(V): kd.insert(c,i)
kd.balance()
def surface_z_at(x,y):
    co,idx,d=kd.find((x,y,ridge_at(x)-0.002)); return V[idx][2]

ANKLE_X=-0.0115; TIP_X=0.0914; HALF_W_MAX=0.006  # matches cut_v_through_lattice.py (+15% toward toe)
ROT_DEG=-1.0
theta=math.radians(ROT_DEG); cos_t=math.cos(theta); sin_t=math.sin(theta)
cx_v=(ANKLE_X+TIP_X)/2.0; cy_v=0.0
def rot(x,y):
    dx=x-cx_v; dy=y-cy_v
    return (cx_v+dx*cos_t-dy*sin_t, cy_v+dx*sin_t+dy*cos_t)

# --- tongue params ---
THICKNESS=0.0010
FRONT_X=TIP_X+0.006   # hinge embed into solid toe (+0.2mm longer at top)
BACK_X =-0.0105        # terminate AT ankle opening edge (was -0.0175, past the shoe)
CONFORM_LEN=0.014
NS=48; NW=11
CONFORM_GAP=0.0008
MAX_DIVE=0.0040       # 4mm dip in the middle (arc toward centerline), returns to flush at ends

# --- crosshatch params ---
HATCH_SPACING=0.0045   # ~4.5mm between parallel lines
RIB_RADIUS=0.00055     # 0.55mm ribbon

def hw_at(x):
    if x> TIP_X:  return 0.0060   # 120% width: tip half 5.0->6.0mm
    if x<ANKLE_X: return 0.0156   # 120% width: ankle half 13.0->15.6mm
    s=(x-ANKLE_X)/(TIP_X-ANKLE_X); return 0.0156+(0.0060-0.0156)*s

def sstep(a,b,x):
    t=max(0.0,min(1.0,(x-a)/(b-a) if b!=a else 0.0)); return t*t*(3-2*t)

def spine_z(x):
    # conform at tip, dip in middle (arc), terminate EXACTLY on ridge (top plane) at back
    # offset below ridge: CONFORM_GAP at tip -> MAX_DIVE at mid -> 0 (flush) at back
    s_len=(FRONT_X-x)/(FRONT_X-BACK_X)   # 0 tip -> 1 back
    s_len=max(0.0,min(1.0,s_len))
    # smooth dip: peaks at s=0.5
    dip = MAX_DIVE * (math.sin(math.pi*s_len)**1.5)
    # conform gap only near the tip (first CONFORM_LEN)
    x_conf_start=TIP_X-CONFORM_LEN
    conf = CONFORM_GAP if x>=x_conf_start else CONFORM_GAP*sstep(BACK_X,x_conf_start,x)
    offset = max(dip, conf)
    return ridge_at(x) - offset

def dome_factor(x):
    # visor bend: GENTLE EVEN ARC (constant along length, like a cap visor)
    return 1.0   # uniform gentle curl everywhere

# --- build the curved surface grid (parametric: u along length, v across width) ---
xs=np.linspace(FRONT_X,BACK_X,NS)
yo=np.linspace(-1.0,1.0,NW)
grid=[]
for x in xs:
    x=float(x); yc=cy_at(x); hw=hw_at(x); sz=spine_z(x); df=dome_factor(x)
    row=[]
    for w in yo:
        y_off=hw*w
        z_edge=surface_z_at(x,yc+y_off)
        dome=(ridge_at(x)-z_edge)*df
        z=sz-dome*(w*w)
        yw=yc+y_off
        rx,ry=rot(x,yw)
        row.append(np.array([rx,ry,z],dtype=float))
    grid.append(row)
grid=np.array(grid)   # (NS, NW, 3)

# --- bilinear sample: (u in [0,1] along length, v in [-1,1] across width) -> 3D ---
L=(FRONT_X-BACK_X)
def sample(u,v):
    # convert u,v to fractional grid indices
    fu=u*(NS-1); si=int(fu); sf=fu-si
    if si>=NS-1: si=NS-2; sf=1.0
    fv=(v+1.0)*0.5*(NW-1); wi=int(fv); wf=fv-wi
    if wi>=NW-1: wi=NW-2; wf=1.0
    p00=grid[si][wi]; p01=grid[si][wi+1]; p10=grid[si+1][wi]; p11=grid[si+1][wi+1]
    return (p00*(1-sf)+p10*sf)*(1-wf) + (p01*(1-sf)+p11*sf)*wf

# --- generate diagonal crosshatch lines in (u,v) space, CLIPPED to superellipse rim ---
# family A: u + v*aspect = k   (diagonal /)
# family B: u - v*aspect = k   (diagonal \)
# aspect = width/length so diagonals are ~45deg in real space
aspect=2.0*hw_at((FRONT_X+BACK_X)/2)/(FRONT_X-BACK_X)
SUP_N=2.5   # MUST match the rim superellipse exponent below

def inside_rim(u, v):
    # matches the centered rim: |2u-1|^(2*SUP_N) + |v|^(2*SUP_N) <= 1
    return abs(2*u-1)**(2*SUP_N) + abs(v)**(2*SUP_N) <= 1.0

def gen_line(k, sign):
    # walk v, compute u, keep only points inside the rim, split into segments
    pts=[]
    v=-1.0
    segments=[]; cur=[]
    while v<=1.001:
        if sign>0:
            u=k - v*aspect
        else:
            u=k + v*aspect
        if 0.0<=u<=1.0 and inside_rim(u,v):
            cur.append(sample(u,v))
        else:
            if len(cur)>=2: segments.append(cur)
            cur=[]
        v+=0.01
    if len(cur)>=2: segments.append(cur)
    return segments

lines=[]
# family A
k_min_A=0.0 - 1.0*aspect; k_max_A=1.0 - (-1.0)*aspect
nA=int((k_max_A-k_min_A)/(HATCH_SPACING/L))+1
for i in range(nA+1):
    k=k_min_A + i*(k_max_A-k_min_A)/max(nA,1)
    lines += gen_line(k, +1)
# family B (SAME range as A)
kB_min=-aspect; kB_max=1.0+aspect
nB=int((kB_max-kB_min)/(HATCH_SPACING/L))+1
for i in range(nB+1):
    k=kB_min + i*(kB_max-kB_min)/max(nB,1)
    lines += gen_line(k, -1)

print('crosshatch lines: family A=%d, family B=%d, total=%d' % (
    nA+1, nB+1, len(lines)))

# --- border rim: two smooth closed loops with gently rounded corners (no hard edges) ---
RIM_RADIUS_FACTOR=1.8   # rim is 1.8x the crosshatch ribbon
SUP_N=2.5               # superellipse exponent (gentle corner rounding); higher=sharper
SUP_NS=200              # samples per loop

def superellipse_loop(au_frac, av, n, nsamp):
    # closed rounded-rectangle loop in (u,v) where u in [0,1], v in [-1,1].
    # CENTERED at u=0.5 with u-radius 0.5*au_frac so u NEVER goes negative.
    pts=[]
    for k in range(nsamp):
        t=2.0*math.pi*k/nsamp
        ct=math.cos(t); st=math.sin(t)
        u=0.5 + 0.5*au_frac*(math.copysign(abs(ct)**(1.0/n), ct) if abs(ct)>1e-12 else 0.0)
        v=av*(math.copysign(abs(st)**(1.0/n), st) if abs(st)>1e-12 else 0.0)
        pts.append(sample(u, v))
    return pts

rim_lines=[]   # (pts, radius_factor)
# OUTER rib: full boundary (u 0->1), corners gently rounded
rim_lines.append((superellipse_loop(1.0, 1.0, SUP_N, SUP_NS), RIM_RADIUS_FACTOR))
# INNER rib: sides parallel at v=+/-0.90, front ~1mm back (au_frac=0.99)
rim_lines.append((superellipse_loop(0.99, 0.90, SUP_N, SUP_NS), RIM_RADIUS_FACTOR))
print('border rims: 2 smooth closed loops (superellipse n=%.1f, centered u=0.5)' % SUP_N)

# --- build as a single curve object with bevel ---
crv=bpy.data.curves.new('Tongue','CURVE')
crv.dimensions='3D'; crv.bevel_depth=RIB_RADIUS; crv.bevel_resolution=2
crv.use_fill_caps=False; crv.resolution_u=8   # no caps: eliminates endpoint blobs at rim junctions
for pts in lines:
    sp=crv.splines.new('POLY'); sp.points.add(len(pts)-1)
    for i,p in enumerate(pts):
        sp.points[i].co=(float(p[0]),float(p[1]),float(p[2]),1.0)
        sp.points[i].radius=1.0
# rim (thicker via per-point radius) - closed loops, cyclic
for pts, rf in rim_lines:
    sp=crv.splines.new('POLY'); sp.use_cyclic_u=True; sp.points.add(len(pts)-1)
    for i,p in enumerate(pts):
        sp.points[i].co=(float(p[0]),float(p[1]),float(p[2]),1.0)
        sp.points[i].radius=rf
tobj=bpy.data.objects.new('Tongue',crv)
bpy.context.collection.objects.link(tobj); tobj.parent=boot

# --- ROTATE tongue: pivot near tip, back dives down into cavity, tip lifts into wall ---
# negative angle = back(down) / tip(up); positive = opposite
ROT_PIVOT_X = FRONT_X - 0.015   # 15mm back from attachment tip
ROT_PIVOT_Z = spine_z(ROT_PIVOT_X)
ROT_ANGLE_DEG = -4.0           # KNOB: final angle (backed off 2deg from -6; tongue tip was poking ~0.05mm through dorsal after elongation)
theta_r = math.radians(ROT_ANGLE_DEG)
rct = math.cos(theta_r); rst = math.sin(theta_r)
for sp in crv.splines:
    for p in sp.points:
        dx = p.co[0] - ROT_PIVOT_X
        dz = p.co[2] - ROT_PIVOT_Z
        p.co[0] = ROT_PIVOT_X + dx*rct + dz*rst
        p.co[2] = ROT_PIVOT_Z - dx*rst + dz*rct

def rot_z(x, z):
    dx=x-ROT_PIVOT_X; dz=z-ROT_PIVOT_Z
    return ROT_PIVOT_Z - dx*rst + dz*rct
print('=== TONGUE BUILT + ROTATED (thatched crosshatch, no hinge) ===')
print('rotation: pivot x=%.0fmm z=%.1fmm, angle=%.1fdeg' % (ROT_PIVOT_X*1000, ROT_PIVOT_Z*1000, ROT_ANGLE_DEG))
print('  tip   z: %.1f -> %.1fmm (lift into wall)' % (spine_z(FRONT_X)*1000, rot_z(FRONT_X, spine_z(FRONT_X))*1000))
print('  back  z: %.1f -> %.1fmm (dive into cavity)' % (spine_z(BACK_X)*1000, rot_z(BACK_X, spine_z(BACK_X))*1000))
print('surface: %d stations x %d across, curved (conform+dome)' % (NS,NW))
print('crosshatch spacing ~%.1fmm, ribbon %.2fmm' % (HATCH_SPACING*1000, RIB_RADIUS*1000))
print('width: %.0fmm hinge -> %.0fmm ankle' % (2*hw_at(FRONT_X)*1000, 2*hw_at(BACK_X)*1000))
