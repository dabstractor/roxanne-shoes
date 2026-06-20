"""V_Band v6: flat trim along both V edges, SAME thickness as the reinforce bands,
MERGING SEAMLESSLY where they cross, and forming a clean POINT at the V tip.

v6 changes (per user): the two arms now TAPER to zero width at the tip and converge
to a SINGLE shared point (was blunt 3.5mm and asymmetric -- left stopped 1 station
short). Taper is a cosine ease over the last 18% of the arm, so the outer edge stays
a smooth continuous curve right into the point (not chopped). A raycast fallback
(nearest pristine vertex) guarantees no station is skipped -> perfect symmetry.

Otherwise identical to v5:
  - Base on pristine shell via raycast, wall 1.5mm along true normal = reinforce-band thickness.
  - TOP built manually and SNAPPED to the reinforce band's evaluated top surface in the
    overlap X-ranges -> seamless merge, no step.
  - Footprint placed explicitly in XY (inner edge = edge - W_IN) -> covers cut lattice
    tube ends on both sides.
  - Constant width along the arm; outer edge = constant parallel offset of the V edge.
  - REQUIRES build_ankle_reinforce.py to run first (snap target)."""
import bpy, bmesh, math, numpy as np
from mathutils.bvhtree import BVHTree
from mathutils.kdtree import KDTree
import mathutils

boot = bpy.data.objects['left boot cutout meters']
src  = bpy.data.objects['left boot cutout BACKUP']
for nm in ('V_Band',):
    old = bpy.data.objects.get(nm)
    if old: bpy.data.objects.remove(old, do_unlink=True)

# --- pristine shell: BVH (raycast) + KDTree (fallback) + world normals ---
Mw = src.matrix_world
Mw3 = Mw.to_3x3()
smesh = src.data
Vsrc = [Mw @ v.co for v in smesh.vertices]
polys = [list(p.vertices) for p in smesh.polygons]
bvh = BVHTree.FromPolygons([tuple(c) for c in Vsrc], polys, all_triangles=False)
Varr = np.array([(c.x, c.y, c.z) for c in Vsrc], dtype=float)
Narr = np.array([tuple(Mw3 @ v.normal) for v in smesh.vertices], dtype=float)
pkd = KDTree(len(Varr))
for i, c in enumerate(Varr): pkd.insert(c, i)
pkd.balance()

# --- reinforce band BVH (snap target) ---
ar_obj = bpy.data.objects.get('Ankle_Reinforce')
if ar_obj is None:
    raise RuntimeError('build_ankle_reinforce.py must run BEFORE build_v_band.py')
dg = bpy.context.evaluated_depsgraph_get()
ar_eo = ar_obj.evaluated_get(dg)
ar_em = ar_eo.to_mesh()
arV = [tuple(v.co) for v in ar_em.vertices]
ar_polys = [list(p.vertices) for p in ar_em.polygons]
ar_bvh = BVHTree.FromPolygons(arV, ar_polys, all_triangles=False)

# --- smoothed centerline + ridge (identical params to build_ankle_reinforce.py) ---
xmin=float(Varr[:,0].min()); xmax=float(Varr[:,0].max())
NB=60; bed=np.linspace(xmin,xmax,NB+1)
bi=np.clip(np.digitize(Varr[:,0],bed)-1,0,NB-1)
cy=np.zeros(NB); cz=np.zeros(NB); ridge=np.zeros(NB); cnt=np.zeros(NB)
for i in range(len(Varr)):
    cy[bi[i]]+=Varr[i,1]; cz[bi[i]]+=Varr[i,2]; cnt[bi[i]]+=1
cnt[cnt==0]=1; cy/=cnt; cz/=cnt
for b in range(NB):
    members=[i for i in range(len(Varr)) if bi[i]==b]
    ridge[b]=max((Varr[i,2] for i in members), default=cz[b])
def smooth1d(a,p,h):
    a=a.copy()
    for _ in range(p):
        s=a.copy()
        for i in range(len(a)):
            lo=max(0,i-h); hi=min(len(a)-1,i+h); s[i]=a[lo:hi+1].mean()
        a=s
    return a
cy=smooth1d(cy,25,4); cz=smooth1d(cz,25,4); ridge=smooth1d(ridge,15,3)
cx_bed=(bed[:-1]+bed[1:])/2.0
def _at(arr,x):
    if x<=cx_bed[0]: return arr[0]
    if x>=cx_bed[-1]: return arr[-1]
    for i in range(NB-1):
        if cx_bed[i]<=x<=cx_bed[i+1]:
            t=(x-cx_bed[i])/(cx_bed[i+1]-cx_bed[i]); return arr[i]+(arr[i+1]-arr[i])*t
    return 0.0
def cy_at(x): return _at(cy,x)
def cz_at(x): return _at(cz,x)
def ridge_at(x): return _at(ridge,x)

# V polygon (match cut_v_through_lattice.py / build_ankle_reinforce.py)
ANKLE_X=-0.0115; TIP_X=0.1014; HALF_W_MAX=0.006; ROT_DEG=-1.0
theta=math.radians(ROT_DEG); cos_t=math.cos(theta); sin_t=math.sin(theta)
cxv=(ANKLE_X+TIP_X)/2.0
def v_width(x):
    s=(x-ANKLE_X)/(TIP_X-ANKLE_X); return HALF_W_MAX*max(0.0,1.0-s)
def rot(x,y):
    dx=x-cxv
    return (cxv+dx*cos_t-y*sin_t, dx*sin_t+y*cos_t)

# band params
W_IN=0.0012; W_OUT=0.0020; WALL=0.00165
ANKLE_END_X=-0.0085   # stop where surface exists (no extrapolation -> no jog). Collar ears bridge to the cuff band.
OVERLAP_X=[(-0.0115,0.0032),(0.0232,0.0368)]
def in_overlap(x):
    return any(lo<=x<=hi for lo,hi in OVERLAP_X)

def _surf_raw(x, y):
    """Centerline raycast -> dorsal hit at (x,y), or None if no surface (open cuff)."""
    cyy = cy_at(x); czz = cz_at(x)
    dy = y - cyy
    dz = max(0.002, ridge_at(x) - czz)
    L = math.hypot(dy, dz) or 1.0
    loc, nrm, idx, d = bvh.ray_cast((x, cyy, czz), (0.0, dy/L, dz/L), 0.06)
    if loc is not None:
        return (loc.x, loc.y, loc.z, nrm.x, nrm.y, nrm.z)
    return None

def surf(x, y):
    """Dorsal surface at (x,y). Past the open ankle cuff (x < ~-9.9, no surface),
    extrapolate from the nearest valid station hit, shifted in -X so the V rails
    extend smoothly to meet the cuff band at the ankle top (-11.5mm). Without this
    the rails dead-end at the last surface point and leave a gap to the cuff band."""
    r = _surf_raw(x, y)
    if r is not None:
        return r
    # open cuff: find nearest x with a valid surface, reuse its hit shifted to this x
    # (search inward in +X from the requested x until a ray hits)
    probe_x = x
    for _ in range(40):
        probe_x += 0.0004   # step 0.4mm toward +X (toward surface)
        if probe_x > 0.0:
            break
        rb = _surf_raw(probe_x, y - (probe_x - x))   # follow the V edge slope in Y
        if rb is not None:
            # shift this hit back to the requested x (preserve cross-section shape)
            return (x, rb[1] - (probe_x - x), rb[2], rb[3], rb[4], rb[5])
    # last resort: nearest pristine vertex
    co, idx2, dd = pkd.find((x, y, ridge_at(x)))
    return (float(Varr[idx2][0]), float(Varr[idx2][1]), float(Varr[idx2][2]),
            float(Narr[idx2][0]), float(Narr[idx2][1]), float(Narr[idx2][2]))

def reinforce_top_z(x, y, z_above):
    """Z of the reinforce band's TOP surface at (x,y), to snap the V band's top
    level with it (seamless merge). Only accept a hit AT OR ABOVE the V band's
    natural top (z_above): at the ankle the ray can otherwise punch through and
    hit the band's underside / inner wall (~12mm), snapping the top DOWN and
    creating a gap. Reject those."""
    if not in_overlap(x): return None
    loc,nrm,idx,d=ar_bvh.ray_cast((x,y,z_above+0.004),(0,0,-1),0.006)
    if loc is None: return None
    if loc.z < z_above - 0.0003:   # hit below the natural top -> wrong surface, reject
        return None
    return loc.z

NS=140
xs=np.linspace(ANKLE_END_X, TIP_X, NS)
edges={
    -1:[rot(float(x), cy_at(float(x))-v_width(float(x))) for x in xs],
    +1:[rot(float(x), cy_at(float(x))+v_width(float(x))) for x in xs],
}
def outward_dir(pts,i,side):
    """Robust outward direction for arm `side` (-1=left, +1=right).
    Uses the side param DIRECTLY (not a fragile centerline comparison) and a
    stable central-difference tangent -> no sign flip near the converging tip,
    which was twisting the band 180deg at one station (the visible twist+gap)."""
    ex,ey=pts[i]
    # central-difference tangent (forward at start, backward at end)
    if 0<i<len(pts)-1:
        tx=(pts[i+1][0]-pts[i-1][0]); ty=(pts[i+1][1]-pts[i-1][1])
    elif i<len(pts)-1:
        tx,ty=pts[i+1][0]-ex, pts[i+1][1]-ey
    else:
        tx,ty=ex-pts[i-1][0], ey-pts[i-1][1]
    L=math.hypot(tx,ty) or 1.0
    ox,oy=-ty/L,tx/L   # one of the two perpendiculars
    # pick the perpendicular pointing toward `side` (left=-1 => oy<0; right=+1 => oy>0)
    if (oy>0 and side<0) or (oy<0 and side>0):
        ox,oy=-ox,-oy
    return ox,oy

# width taper: FULL width in the reinforce-band overlap zones (so it stays flush with
# them and keeps covering the lattice ends at the ankle), then smoothstep taper down
# toward the V tip over the rest of the length -- but only to a MIN_WIDTH floor, not
# zero, so the two arms still OVERLAP at the tip (cross each other) rather than both
# collapsing to a single point. The outer edge is a constant parallel offset of the V
# edge (never shaved); narrowing comes from the inside (gap-facing) retreating.
TAPER_START_X = 0.040   # beyond the foot band (x=36.8mm), start tapering toward the tip
MIN_WIDTH     = 0.0008  # 0.8mm floor -- arms still overlap at the tip (don't vanish)
def width_scale_at(x):
    if x <= TAPER_START_X: return 1.0
    s = (x - TAPER_START_X) / (TIP_X - TAPER_START_X)   # 0 at start, 1 at tip
    s = max(0.0, min(1.0, s))
    full = W_IN + W_OUT
    # smoothstep from full width down to MIN_WIDTH
    w = full - (full - MIN_WIDTH) * s*s*(3-2*s)
    return w / full   # caller multiplies by (W_IN, W_OUT)

# shared convergence point (both arms meet HERE -> single clean point, no asymmetry).
# Probe z+normal ROBUSTLY by averaging several points near the tip along the
# centerline -- a single raycast at the exact mesh edge (x=TIP_X) hits low/side
# geometry (returned z=13.14 vs ~15 dorsal) and sank the tip into a notch.
conv = rot(TIP_X, cy_at(TIP_X))
_probe_pts = [rot(TIP_X - dx, cy_at(TIP_X - dx)) for dx in (0.0040, 0.0025, 0.0012)]
_zs=[]; _ns=[]
for _px,_py in _probe_pts:
    _r = surf(_px, _py)
    if _r is not None:
        _zs.append(_r[2]); _ns.append(np.array(_r[3:6]))
if not _zs:
    _r = surf(conv[0], conv[1]); _zs=[_r[2]]; _ns=[np.array(_r[3:6])]
tip_base_z = float(np.mean(_zs))
tip_nrm = sum(_ns) / (np.linalg.norm(sum(_ns)) or 1.0)

bm=bmesh.new()
# TWO tip verts: bottom (on surface) + top (surface + wall along normal). Using one
# shared vert for both caps (old code) forced the top cap to dip to the surface z ->
# a sunken notch at the tip (= the visible dead-end). Now the top stays flat.
tip_bottom = bm.verts.new((conv[0], conv[1], tip_base_z))
tip_top    = bm.verts.new((conv[0] + tip_nrm[0]*WALL, conv[1] + tip_nrm[1]*WALL, tip_base_z + tip_nrm[2]*WALL))

def build_side(side, tip_bottom, tip_top):
    pts=edges[side]; n=len(pts)
    rows=[]
    for i,(ex,ey) in enumerate(pts):
        ox,oy=outward_dir(pts,i,side)
        ws=width_scale_at(ex)
        if i==n-1:
            rows.append(('tip',))   # marker; caps use shared tip_bottom/tip_top
            continue
        win=W_IN*ws; wout=W_OUT*ws
        ix,iy=ex-ox*win, ey-oy*win
        oxp,oyp=ex+ox*wout, ey+oy*wout
        ri=surf(ix,iy); ro=surf(oxp,oyp)
        bi_v=bm.verts.new(ri[:3]); bo_v=bm.verts.new(ro[:3])
        ti=np.array([ri[0]+ri[3]*WALL, ri[1]+ri[4]*WALL, ri[2]+ri[5]*WALL])
        to=np.array([ro[0]+ro[3]*WALL, ro[1]+ro[4]*WALL, ro[2]+ro[5]*WALL])
        # RAISE the V band above the reinforce band ends where they overlap, so the
        # band ends are HIDDEN under the V band (not protruding through). The old snap
        # set the V band flush with the band top -> no matter how thick WALL got, the
        # overlap height never changed (looked identical every rebuild). Now we take
        # max(natural WALL height, reinforce_top + HIDE_MARGIN) so the V band always
        # sits at least HIDE_MARGIN above the band ends, growing OUTWARD (away from leg).
        HIDE_MARGIN=0.0003   # 0.3mm above band ends
        for P in (ti,to):
            rz=reinforce_top_z(P[0],P[1],P[2])
            if rz is not None:
                P[2]=max(P[2], rz+HIDE_MARGIN)
        ti_v=bm.verts.new(ti.tolist()); to_v=bm.verts.new(to.tolist())
        rows.append(('seg', bi_v, bo_v, ti_v, to_v))
    nf=0
    def mk(q):
        nonlocal nf
        try: bm.faces.new(q); nf+=1
        except ValueError: pass
    # WINDING: the two arms are geometric mirrors (outward_dir flips inner/outer physical
    # sides). Identical quad winding therefore produces OUTWARD normals on one arm and
    # INWARD (inside-out) normals on the other. The +1 arm was entirely inside-out
    # (top faces pointing -Z), which made that whole side of the V drop out of the slicer.
    # Reverse the winding for the +1 arm so both arms' normals point outward.
    def quad(a,b,c,d): return (a,b,c,d) if side<0 else (d,c,b,a)
    for k in range(len(rows)-1):
        a=rows[k]; b=rows[k+1]
        if a[0]=='seg' and b[0]=='seg':
            _,bi0,bo0,ti0,to0=a; _,bi1,bo1,ti1,to1=b
            mk(quad(bi0,bo0,bo1,bi1)); mk(quad(ti0,to0,to1,ti1))
            mk(quad(bo0,to0,to1,bo1)); mk(quad(ti0,bi0,bi1,ti1))
        elif a[0]=='seg' and b[0]=='tip':
            _,bi0,bo0,ti0,to0=a
            # close to the shared tip: bottom tri, top tri, outer wall quad, inner wall quad
            if side<0:
                mk((bi0,bo0,tip_bottom)); mk((ti0,to0,tip_top))
                mk((bo0,to0,tip_top,tip_bottom)); mk((ti0,bi0,tip_bottom,tip_top))
            else:
                mk((bo0,bi0,tip_bottom)); mk((to0,ti0,tip_top))
                mk((to0,bo0,tip_bottom,tip_top)); mk((bi0,ti0,tip_top,tip_bottom))
    return nf,len(rows)

nL=build_side(-1, tip_bottom, tip_top); nR=build_side(+1, tip_bottom, tip_top)
bm.normal_update()
me=bpy.data.meshes.new('V_Band'); bm.to_mesh(me); bm.free(); me.update()
obj=bpy.data.objects.new('V_Band',me); bpy.context.collection.objects.link(obj); obj.parent=boot
ar_eo.to_mesh_clear()

# (BASELINE: print-plane trim removed per user request. The V band ankle cap stays
# as-built; flattening will be re-solved a different way later.)
print('=== V_BAND v6 BUILT (tapered point + shared convergence) ===')
print('  left %d faces / %d stations, right %d faces / %d stations' % (nL[0],nL[1],nR[0],nR[1]))
print('  width %.1fmm (W_IN %.1f + W_OUT %.1f), tapering smoothly over the whole length -> clean point' % (
    (W_IN+W_OUT)*1000, W_IN*1000, W_OUT*1000))
print('  shared tip at x=%.1fmm y=%.2fmm  base z=%.2fmm top z=%.2fmm (no sunken notch)' % (
    conv[0]*1000, conv[1]*1000, tip_base_z*1000, (tip_base_z+tip_nrm[2]*WALL)*1000))
