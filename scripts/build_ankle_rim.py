"""STEP 6 — Ankle_Rim: a SMALL (~1mm) flat finishing rim around the ankle opening
so the lattice tubes terminate into something clean (and the ankle-down first print
layer is a tidy ring instead of dangling open tube ends). NOT the old stiff cuff band
(that was ~12mm wide + a lace anchor) — this is a 1mm-thick, ~1mm-wide lip right at
the opening. Follows the opening cross-section (with the V gap, terminating at the V
rails), built with the SAME centerline-raycast technique as build_ankle_reinforce.py.

Tunable: RIM_X (x-range), RIM_STATION (resolution), Solidify wall/offset."""
import bpy, bmesh, math, numpy as np
from mathutils.bvhtree import BVHTree

boot = bpy.data.objects['left boot cutout meters']
src  = bpy.data.objects['left boot cutout BACKUP']
mesh = src.data
for nm in ('Ankle_Rim',):
    old = bpy.data.objects.get(nm)
    if old: bpy.data.objects.remove(old, do_unlink=True)

# --- BVH of the pristine shell in WORLD space (for raycasting) ---
Mw = src.matrix_world
V = np.array([tuple(Mw @ v.co) for v in mesh.vertices], dtype=float)
polys = [list(p.vertices) for p in mesh.polygons]
bvh = BVHTree.FromPolygons([tuple(v) for v in V], polys, all_triangles=False)
xmin = float(V[:,0].min()); xmax = float(V[:,0].max())

# --- smoothed centerline (identical params to build_lattice.py / build_ankle_reinforce.py) ---
NB = 60
bed = np.linspace(xmin, xmax, NB+1)
bi = np.clip(np.digitize(V[:,0], bed)-1, 0, NB-1)
cy = np.zeros(NB); cz = np.zeros(NB); cnt = np.zeros(NB)
for i in range(len(V)):
    cy[bi[i]] += V[i,1]; cz[bi[i]] += V[i,2]; cnt[bi[i]] += 1
cnt[cnt==0] = 1; cy /= cnt; cz /= cnt
def smooth1d(arr, passes, hw):
    a = arr.copy()
    for _ in range(passes):
        s = a.copy()
        for i in range(len(a)):
            lo=max(0,i-hw); hi=min(len(a)-1,i+hw); s[i]=a[lo:hi+1].mean()
        a = s
    return a
cy = smooth1d(cy, 25, 4); cz = smooth1d(cz, 25, 4)
cx_bed = (bed[:-1]+bed[1:])/2.0
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

# --- V polygon + collar polygon (MUST match cut_v_through_lattice.py) ---
ANKLE_X=-0.0115; TIP_X=0.1014; HALF_W_MAX=0.006; DORSAL_Z=0.25; ROT_DEG=-1.0
EAR_MARGIN = 0.0013
th_v=math.radians(ROT_DEG); cos_t=math.cos(th_v); sin_t=math.sin(th_v)
cxv=(ANKLE_X+TIP_X)/2.0; cyv=0.0
def rot(x,y):
    dx=x-cxv; dy=y-cyv
    return (cxv+dx*cos_t-dy*sin_t, cyv+dx*sin_t+dy*cos_t)
def v_width(x):
    s=(x-ANKLE_X)/(TIP_X-ANKLE_X); return HALF_W_MAX*max(0.0,1.0-s)
NS=41
sx=np.linspace(ANKLE_X,TIP_X,NS)
cy_sx=np.array([cy_at(x) for x in sx]); vw_sx=np.array([v_width(x) for x in sx])
left_collar =[rot(float(x), float(cy_sx[i]-vw_sx[i]+EAR_MARGIN)) for i,x in enumerate(sx)]
right_collar=[rot(float(x), float(cy_sx[i]+vw_sx[i]-EAR_MARGIN)) for i,x in enumerate(sx)]
poly_collar = left_collar + list(reversed(right_collar))
def point_in_poly_collar(px,py):
    inside=False; n=len(poly_collar); j=n-1
    for i in range(n):
        xi,yi=poly_collar[i]; xj,yj=poly_collar[j]
        if ((yi>py)!=(yj>py)) and (px<(xj-xi)*(py-yi)/((yj-yi) or 1e-12)+xi):
            inside=not inside
        j=i
    return inside

# --- rim strip params ---
RIM_XLO = -0.0105   # -10.5mm: lip ~0.5mm past the ankle opening edge (~-9.98mm)
RIM_XHI = -0.0095   # -9.5mm: ~0.5mm inside where the lattice tubes are (they end ~-10.0mm)
RIM_STATION = 0.00025   # 0.25mm station resolution
NTHETA_DENSE = 144; M_COLS = 48; RAY_MAX = 0.06
xs = np.arange(RIM_XLO, RIM_XHI + 1e-9, RIM_STATION)

def cross_section_arc(x):
    cyy = cy_at(x); czz = cz_at(x); origin = (x, cyy, czz); hits = []
    for k in range(NTHETA_DENSE):
        th = 2.0*math.pi*k/NTHETA_DENSE
        loc, nrm, idx, dist = bvh.ray_cast(origin, (0.0, math.cos(th), math.sin(th)), RAY_MAX)
        if loc is None: continue
        if point_in_poly_collar(loc.x, loc.y) and nrm.z > DORSAL_Z: continue
        hits.append((th, np.array([loc.x, loc.y, loc.z], dtype=float)))
    if len(hits) < 8: return None
    hits.sort(key=lambda t: t[0]); n = len(hits); thetas = [h[0] for h in hits]
    gaps = [ (thetas[(i+1)%n] - thetas[i]) % (2*math.pi) for i in range(n) ]
    i_gap = max(range(n), key=lambda i: gaps[i])
    order = [(i_gap+1+k) % n for k in range(n)]
    return [hits[i][1] for i in order]

def extend_arc_to_rail(arc, x):
    if arc is None or len(arc) < 4: return arc
    cyy = cy_at(x)
    _, yt_left  = rot(x, cyy - v_width(x) + EAR_MARGIN)
    _, yt_right = rot(x, cyy + v_width(x) - EAR_MARGIN)
    def _ext(pt_end, yt, go_neg):
        if go_neg and pt_end[1] <= yt: return None
        if (not go_neg) and pt_end[1] >= yt: return None
        return np.array([pt_end[0], yt, pt_end[2]])
    a0, am1 = arc[0], arc[-1]; pre = app = None
    if a0[1] < am1[1]:
        nl = _ext(a0, yt_left, True);  nr = _ext(am1, yt_right, False)
        if nl is not None: pre = nl
        if nr is not None: app = nr
    else:
        nl = _ext(am1, yt_left, True);  nr = _ext(a0, yt_right, False)
        if nl is not None: app = nl
        if nr is not None: pre = nr
    out = list(arc)
    if pre is not None: out = [pre] + out
    if app is not None: out = out + [app]
    return out

def resample_arc(points, M):
    pts = np.asarray(points, dtype=float)
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    if s[-1] <= 1e-9: return pts
    t = np.linspace(0.0, s[-1], M)
    return np.stack([np.interp(t, s, pts[:,j]) for j in range(3)], axis=1)

# first pass: collect valid arcs (stations where the surface exists)
valid = []
for x in xs:
    arc = cross_section_arc(float(x))
    if arc is not None: valid.append((float(x), arc))
if not valid:
    raise RuntimeError('No valid cross-section in rim X-range (mesh edge moved?)')
valid_x = np.array([v[0] for v in valid])

# build quad grid; extrapolate open-cuff stations from nearest valid arc (shift in -X only)
bm = bmesh.new(); rows = []
for x in xs:
    x = float(x); arc = cross_section_arc(x)
    if arc is None:
        j = int(np.argmin(np.abs(valid_x - x)))
        base_x, base_arc = valid[j]
        base_centroid = np.mean(base_arc, axis=0)
        arc = [p + np.array([x - base_centroid[0], 0.0, 0.0]) for p in base_arc]
    arc = extend_arc_to_rail(arc, x)
    rp = resample_arc(arc, M_COLS)
    rows.append([bm.verts.new((float(p[0]), float(p[1]), float(p[2]))) for p in rp])
nf = 0
for r in range(len(rows)-1):
    a = rows[r]; b = rows[r+1]
    for j in range(M_COLS-1):
        try: bm.faces.new((a[j], a[j+1], b[j+1], b[j])); nf += 1
        except ValueError: pass
bm.normal_update()
out_mesh = bpy.data.meshes.new('Ankle_Rim')
bm.to_mesh(out_mesh); bm.free(); out_mesh.update()
obj = bpy.data.objects.new('Ankle_Rim', out_mesh)
bpy.context.collection.objects.link(obj); obj.parent = boot

# Wall CENTERED on the surface (offset 0) and wide enough to cover BOTH lattice
# layers fully at the cuff thickness -- outer at +0.448mm, inner at -0.448mm, plus
# tube radius (0.459*CUFF_SCALE). At CUFF_SCALE=2.1 the cuff FUSES into a ~2.8mm
# block, so the rim is 3.0mm (±1.5mm) to cap it with no protrusion. KEEP IN SYNC
# with CUFF_SCALE in build_lattice.py. Tunable: thickness / offset.
mod = obj.modifiers.new('Solidify', 'SOLIDIFY')
mod.thickness = 0.0030
mod.offset = 0.0
mod.use_even_offset = True
mod.use_quality_normals = True

print('=== ANKLE_RIM BUILT ===')
print('  x[%6.2f..%6.2f]mm  %d stations x %d cols  %d quads  (valid surface stations: %d)' % (
    RIM_XLO*1000, RIM_XHI*1000, len(rows), M_COLS, nf, len(valid)))
print('  Solidify wall 3.00mm, centered (offset 0 -> caps both lattice layers +/-1.5mm at CUFF_SCALE=2.3)')
