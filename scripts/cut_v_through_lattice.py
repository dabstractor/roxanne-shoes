"""Cut the V tongue cutout through BOTH the mesh and the already-built lattice curves.
Run AFTER build_lattice.py (which builds on the uncut mesh).
The V rotation is now a free knob (no tear coupling)."""
import bpy, bmesh, math, numpy as np

boot = bpy.data.objects['left boot cutout meters']
mesh = boot.data
if bpy.context.object and bpy.context.object.mode != 'OBJECT':
    bpy.ops.object.mode_set(mode='OBJECT')
for o in bpy.context.view_layer.objects: o.select_set(False)
bpy.context.view_layer.objects.active = boot; boot.select_set(True)

# --- smoothed centerline (identical to build_lattice.py) ---
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

# --- V params ---
ANKLE_X=-0.0115; TIP_X=0.1014; HALF_W_MAX=0.006; DORSAL_Z=0.25  # +23.4mm (~26% of 89.5mm wedge) toward toe for foot entry (+10mm vs prev 0.0914)
ROT_DEG=-1.0   # FREE KNOB now (no tear coupling). -1deg aligns with centerline.

theta=math.radians(ROT_DEG); cos_t=math.cos(theta); sin_t=math.sin(theta)
cx_v=(ANKLE_X+TIP_X)/2.0; cy_v=0.0
def rot(x,y):
    dx=x-cx_v; dy=y-cy_v
    return (cx_v+dx*cos_t-dy*sin_t, cy_v+dx*sin_t+dy*cos_t)
def v_width(x):
    s=(x-ANKLE_X)/(TIP_X-ANKLE_X); return HALF_W_MAX*max(0.0,1.0-s)

# build rotated piecewise V polygon
NS=41
sx=np.linspace(ANKLE_X,TIP_X,NS)
cy_sx=np.array([cy_at(x) for x in sx])
vw_sx=np.array([v_width(x) for x in sx])
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

# === 1. CUT V ON MESH (bisect method) ===
def plane_for(p1,p2):
    dx=p2[0]-p1[0]; dy=p2[1]-p1[1]; L=math.hypot(dx,dy) or 1.0
    return ((p1[0],p1[1],0.0),(-dy/L,dx/L,0.0))
segs=[]
for i in range(NS-1):
    segs.append(plane_for(left_pts[i],left_pts[i+1]))
    segs.append(plane_for(right_pts[i],right_pts[i+1]))
segs.append(((ANKLE_X,0,0),(1.0,0.0,0.0)))
bm=bmesh.new(); bm.from_mesh(mesh)
for co,no in segs:
    dorsal=[f for f in bm.faces if f.normal.z>DORSAL_Z]
    eset=set()
    for f in dorsal:
        for e in f.edges: eset.add(e)
    if dorsal:
        bmesh.ops.bisect_plane(bm, geom=list(dorsal)+list(eset), plane_co=co, plane_no=no, dist=1e-7, clear_outer=False, clear_inner=False)
bm.faces.ensure_lookup_table()
to_del=[f for f in bm.faces if f.normal.z>DORSAL_Z and point_in_poly(f.calc_center_median().x, f.calc_center_median().y)]
bmesh.ops.delete(bm, geom=to_del, context='FACES')
loose=[v for v in bm.verts if not v.link_faces]
if loose: bmesh.ops.delete(bm, geom=loose, context='VERTS')
bm.to_mesh(mesh); bm.free(); mesh.update()
print('V cut on mesh: %d faces removed, %d verts' % (len(to_del), len(mesh.vertices)))

# recompute gradient on cut mesh (for solidify)
xs=[v.co.x for v in mesh.vertices]; zs=[v.co.z for v in mesh.vertices]
xmin2,xmax2=min(xs),max(xs); zmin2,zmax2=min(zs),max(zs)
def ss2(a,b,x):
    t=max(0.0,min(1.0,(x-a)/(b-a) if b!=a else 0.0)); return t*t*(3-2*t)
vg=boot.vertex_groups.get('gradient')
if vg is None: vg=boot.vertex_groups.new(name='gradient')
for v in mesh.vertices:
    fx=(v.co.x-xmin2)/(xmax2-xmin2) if xmax2!=xmin2 else 0
    fz=(v.co.z-zmin2)/(zmax2-zmin2) if zmax2!=zmin2 else 0
    base=ss2(0.45,0.92,fx); dorsal=0.3+0.7*ss2(0.30,0.90,fz)
    vg.add([v.index], max(0.0,min(1.0,0.15+(1-0.15)*base*dorsal)), 'REPLACE')

# === 2. TRIM LATTICE CURVES THROUGH V ===
def trim_curve_obj(name):
    obj=bpy.data.objects.get(name)
    if obj is None: return 0
    old_crv=obj.data
    new_crv=bpy.data.curves.new(name+'_trimmed','CURVE')
    new_crv.dimensions='3D'; new_crv.bevel_depth=old_crv.bevel_depth
    new_crv.bevel_resolution=old_crv.bevel_resolution; new_crv.use_fill_caps=old_crv.use_fill_caps
    new_crv.resolution_u=old_crv.resolution_u
    n_kept=0; n_split=0
    for sp in old_crv.splines:
        pts=[(p.co[0],p.co[1],p.co[2],p.radius) for p in sp.points]
        if len(pts)<2:
            continue
        # classify each point: inside V AND on dorsal (z>0). Sole (z<0) is never trimmed.
        flags=[point_in_poly(p[0],p[1]) and p[2]>0.0 for p in pts]
        # split into outside segments
        segments=[]; cur=[]
        for i,f in enumerate(flags):
            if not f:
                cur.append(pts[i])
            else:
                if len(cur)>=2: segments.append(cur)
                cur=[]
        if len(cur)>=2: segments.append(cur)
        if len(segments)>1: n_split+=1
        for seg in segments:
            if len(seg)<2: continue
            nsp=new_crv.splines.new('POLY'); nsp.points.add(len(seg)-1)
            for j,p in enumerate(seg):
                nsp.points[j].co=(p[0],p[1],p[2],1.0); nsp.points[j].radius=p[3]
            n_kept+=1
    # replace object data
    n_old=len(old_crv.splines) if old_crv else 0
    obj.data=new_crv
    bpy.data.curves.remove(old_crv)
    print('%s: %d splines -> %d after V trim (%d split)' % (name, n_old, n_kept, n_split))
    return n_kept

trim_curve_obj('Lattice_OUTER')
trim_curve_obj('Lattice_INNER')

boot.hide_set(True)
print('=== V CUT THROUGH LATTICE COMPLETE (ROT_DEG=%.1f) ===' % ROT_DEG)
