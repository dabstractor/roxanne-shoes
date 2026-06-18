"""STEP 5 — Ankle reinforcement: two SOLID bands on the outer layer for lace/clip
anchors. Built as a CLEAN PARAMETRIC GRID (not extracted mesh faces):
  - For each x-station in the strip, raycast a dense fan from the smoothed
    centerline outward against the pristine closed shell (true surface points).
  - Drop the dorsal hits that fall inside the V polygon (keeps the tongue
    opening clear), then take the single bottom arc (complement of the V gap).
  - Resample every cross-section to a FIXED point count by arc length, so every
    row is identical length and quads connect cleanly with zero raggedness.
  - Solidify outward for a real wall.
Regular grid + true raycast normals => no per-vertex-normal spikes, no jagged
boundaries. This is the same parametric-sampling technique build_tongue.py uses.
"""
import bpy, bmesh, math, numpy as np
from mathutils.bvhtree import BVHTree

boot = bpy.data.objects['left boot cutout meters']   # live (cut) mesh; parent only
src  = bpy.data.objects['left boot cutout BACKUP']    # pristine closed shell we sample
mesh = src.data

for nm in ('Ankle_Reinforce',):
    old = bpy.data.objects.get(nm)
    if old: bpy.data.objects.remove(old, do_unlink=True)

# --- BVH of the pristine shell in WORLD space (for raycasting) ---
Mw = src.matrix_world
V = np.array([tuple(Mw @ v.co) for v in mesh.vertices], dtype=float)
polys = [list(p.vertices) for p in mesh.polygons]
bvh = BVHTree.FromPolygons([tuple(v) for v in V], polys, all_triangles=False)

xmin = float(V[:,0].min()); xmax = float(V[:,0].max())

# --- smoothed centerline (identical to build_lattice.py / cut_v) ---
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

# --- V polygon (MUST match cut_v_through_lattice.py / build_tongue.py) ---
ANKLE_X=-0.0115; TIP_X=0.0914; HALF_W_MAX=0.006; DORSAL_Z=0.25
ROT_DEG=-1.0
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
left_pts =[rot(float(x), float(cy_sx[i]-vw_sx[i])) for i,x in enumerate(sx)]
right_pts=[rot(float(x), float(cy_sx[i]+vw_sx[i])) for i,x in enumerate(sx)]
poly = left_pts + list(reversed(right_pts))
def point_in_poly(px,py):
    inside=False; n=len(poly); j=n-1
    for i in range(n):
        xi,yi=poly[i]; xj,yj=poly[j]
        if ((yi>py)!=(yj>py)) and (px<(xj-xi)*(py-yi)/((yj-yi) or 1e-12)+xi):
            inside=not inside
        j=i
    return inside

# --- bands (meters) ---
STRIPS = [
    (-0.00980,  0.0032, 'cuff'),   # top: ~13mm wide (was 8.8), anchored at ankle opening, extends toward foot
    ( 0.02325, 0.03675, 'foot'),  # bottom: ~13.5mm wide (was 9.0), raised toward ankle by 4.5mm (50% of old width); center now at waist->bulge transition (x=30)
]

WALL         = 0.00150   # 1.5mm solid wall (caps + embeds the outer lattice tubes, peaks ~0.96mm)
NTHETA_DENSE = 144       # angular samples per cross-section (every 2.5deg)
M_COLS       = 48        # resampled points per cross-section (fixed -> clean quads)
RAY_MAX      = 0.06      # 60mm reach from centerline

def cross_section_arc(x):
    """Raycast a dense fan at this x from the centerline (YZ plane). Return the
    single bottom arc (V opening removed), as an ordered list of 3D points."""
    cyy = cy_at(x); czz = cz_at(x)
    origin = (x, cyy, czz)
    hits = []   # (theta, point)
    for k in range(NTHETA_DENSE):
        th = 2.0*math.pi*k/NTHETA_DENSE
        loc, nrm, idx, dist = bvh.ray_cast(origin, (0.0, math.cos(th), math.sin(th)), RAY_MAX)
        if loc is None:
            continue
        # skip the dorsal V patch (uncut shell still has a top here; the tongue needs the gap)
        if point_in_poly(loc.x, loc.y) and nrm.z > DORSAL_Z:
            continue
        hits.append((th, np.array([loc.x, loc.y, loc.z], dtype=float)))
    if len(hits) < 8:
        return None
    hits.sort(key=lambda t: t[0])
    # find the largest angular gap between consecutive hits (cyclic) -> that's the V opening.
    # the bottom arc is the COMPLEMENT: start just after the gap, wrap around.
    n = len(hits)
    thetas = [h[0] for h in hits]
    gaps = [ (thetas[(i+1)%n] - thetas[i]) % (2*math.pi) for i in range(n) ]
    i_gap = max(range(n), key=lambda i: gaps[i])
    order = [(i_gap+1+k) % n for k in range(n)]
    return [hits[i][1] for i in order]

def resample_arc(points, M):
    """Resample an ordered open polyline to M evenly arc-length-spaced points."""
    pts = np.asarray(points, dtype=float)
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    total = s[-1]
    if total <= 1e-9:
        return pts
    t = np.linspace(0.0, total, M)
    return np.stack([np.interp(t, s, pts[:,j]) for j in range(3)], axis=1)

# --- build one mesh containing both strips ---
bm = bmesh.new()
total_v = 0; total_f = 0
for (xlo, xhi, label) in STRIPS:
    nX = max(4, int(round((xhi-xlo)/0.0015)) + 1)
    xs = np.linspace(xlo, xhi, nX)
    rows = []   # list of bmv-lists (one per valid station)
    for x in xs:
        arc = cross_section_arc(float(x))
        if arc is None:
            print('    WARN: strip %s x=%.1fmm -> no arc' % (label, x*1000)); continue
        rp = resample_arc(arc, M_COLS)
        rows.append([bm.verts.new((float(p[0]), float(p[1]), float(p[2]))) for p in rp])
    nf = 0
    for r in range(len(rows)-1):
        a = rows[r]; b = rows[r+1]
        for j in range(M_COLS-1):
            try:
                bm.faces.new((a[j], a[j+1], b[j+1], b[j])); nf += 1
            except ValueError:
                pass
    total_v += sum(len(r) for r in rows); total_f += nf
    print('  strip %-5s x[%5.1f..%5.1f]mm  %2d stations x %d cols  %d quads' % (
        label, xlo*1000, xhi*1000, len(rows), M_COLS, nf))

bm.normal_update()
out_mesh = bpy.data.meshes.new('Ankle_Reinforce')
bm.to_mesh(out_mesh); bm.free(); out_mesh.update()
obj = bpy.data.objects.new('Ankle_Reinforce', out_mesh)
bpy.context.collection.objects.link(obj); obj.parent = boot

# --- Solidify outward (caps + embeds the outer lattice into a solid band) ---
mod = obj.modifiers.new('Solidify', 'SOLIDIFY')
mod.thickness = WALL
mod.offset = 1.0                # grow fully outward from the surface
mod.use_even_offset = True
mod.use_quality_normals = True

print('=== ANKLE REINFORCE BUILT (raycast parametric grid + solidify) ===')
print('strips:', [(round(lo*1000,1), round(hi*1000,1), lab) for (lo,hi,lab) in STRIPS])
print('base grid: %d verts, %d faces  + Solidify wall %.2fmm outward' % (
    total_v, total_f, WALL*1000))
