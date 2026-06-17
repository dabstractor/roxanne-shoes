import bpy, bmesh, math, numpy as np

boot = bpy.data.objects['left boot cutout meters']
bak = bpy.data.objects.get('left boot cutout BACKUP')
if bpy.context.object and bpy.context.object.mode != 'OBJECT':
    bpy.ops.object.mode_set(mode='OBJECT')
for o in bpy.context.view_layer.objects: o.select_set(False)
bpy.context.view_layer.objects.active = boot; boot.select_set(True)

# restore uncut
assert bak is not None
boot.data = bak.data.copy()
mesh = boot.data
print('restored: verts', len(mesh.vertices))

# smoothed centerline (identical to lattice script)
V=np.array([tuple(v.co) for v in mesh.vertices],dtype=float)
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
cy=smooth1d(cy,25,4)
cx_bed=(bed[:-1]+bed[1:])/2.0
def cy_at(x):
    if x<=cx_bed[0]: return cy[0]
    if x>=cx_bed[-1]: return cy[-1]
    for i in range(NB-1):
        if cx_bed[i]<=x<=cx_bed[i+1]:
            t=(x-cx_bed[i])/(cx_bed[i+1]-cx_bed[i]); return cy[i]+(cy[i+1]-cy[i])*t
    return 0.0

# --- V params (same as last good cut) ---
ANKLE_X=-0.0115; TIP_X=0.078; HALF_W_MAX=0.006; DORSAL_Z=0.25

# report centerline Y at tip so we can sanity-check the rotation direction
cy_tip=cy_at(TIP_X); cy_ankle=cy_at(ANKLE_X)
print('centerline Y at tip   (x=%.3f): %.4f mm' % (TIP_X, cy_tip*1000))
print('centerline Y at ankle (x=%.3f): %.4f mm' % (ANKLE_X, cy_ankle*1000))
# current V axis Y (straight): 0 at both ends. Angle from ankle->tip in XY:
cur_dy=0.0; cur_dx=TIP_X-ANKLE_X
cur_ang=math.degrees(math.atan2(cur_dy, cur_dx))
print('current V axis angle (XY): %.4f deg' % cur_ang)

# --- 1 degree CLOCKWISE (top-down view, +Z at you): tip -> +Y, ankle -> -Y ---
# rotate the V's XY footprint by -1 deg about the centroid (XY), standard CW = negative in math convention
ROT_DEG=-2.0
theta=math.radians(ROT_DEG)
cos_t=math.cos(theta); sin_t=math.sin(theta)
# centroid of V footprint (approx midpoint of axis)
cx_v=(ANKLE_X+TIP_X)/2.0; cy_v=0.0
def rot(x,y):
    dx=x-cx_v; dy=y-cy_v
    return (cx_v + dx*cos_t - dy*sin_t, cy_v + dx*sin_t + dy*cos_t)

def v_width(x):
    s=(x-ANKLE_X)/(TIP_X-ANKLE_X); return HALF_W_MAX*max(0.0,1.0-s)

# build ROTATED piecewise boundary
NS=41
sx=np.linspace(ANKLE_X,TIP_X,NS)
cy_sx=np.array([cy_at(x) for x in sx])
vw_sx=np.array([v_width(x) for x in sx])
# rotate each boundary sample point
left_pts =[rot(float(x), float(cy_sx[i]-vw_sx[i])) for i,x in enumerate(sx)]
right_pts=[rot(float(x), float(cy_sx[i]+vw_sx[i])) for i,x in enumerate(sx)]

# for the delete test we need an "inside V" predicate in ROTATED space.
# Build closed polygon (left forward + right backward) and use point-in-polygon.
poly=[]
poly+=left_pts
poly+=list(reversed(right_pts))
def point_in_poly(px,py, poly):
    inside=False
    n=len(poly); j=n-1
    for i in range(n):
        xi,yi=poly[i]; xj,yj=poly[j]
        if ((yi>py)!=(yj>py)) and (px < (xj-xi)*(py-yi)/((yj-yi) or 1e-12)+xi):
            inside=not inside
        j=i
    return inside

# bisect planes from rotated segments
def plane_for(p1,p2):
    dx=p2[0]-p1[0]; dy=p2[1]-p1[1]; L=math.hypot(dx,dy) or 1.0
    return ((p1[0],p1[1],0.0),(-dy/L,dx/L,0.0))
segs=[]
for i in range(NS-1):
    segs.append(plane_for(left_pts[i],left_pts[i+1]))
    segs.append(plane_for(right_pts[i],right_pts[i+1]))
segs.append(((ANKLE_X,0,0),(1.0,0.0,0.0)))  # ankle cutoff (pre-rotation)
print('bisect planes:', len(segs))

bm=bmesh.new(); bm.from_mesh(mesh)
for co,no in segs:
    dorsal=[f for f in bm.faces if f.normal.z>DORSAL_Z]
    eset=set()
    for f in dorsal:
        for e in f.edges: eset.add(e)
    if dorsal:
        bmesh.ops.bisect_plane(bm, geom=list(dorsal)+list(eset), plane_co=co, plane_no=no, dist=1e-7, clear_outer=False, clear_inner=False)
bm.faces.ensure_lookup_table()
to_del=[f for f in bm.faces if f.normal.z>DORSAL_Z and point_in_poly(f.calc_center_median().x, f.calc_center_median().y, poly)]
bmesh.ops.delete(bm, geom=to_del, context='FACES')
loose=[v for v in bm.verts if not v.link_faces]
if loose: bmesh.ops.delete(bm, geom=loose, context='VERTS')
bm.to_mesh(mesh); bm.free(); mesh.update()
print('faces removed:', len(to_del), '| verts', len(mesh.vertices), 'faces', len(mesh.polygons))

# recompute gradient
xs=[v.co.x for v in mesh.vertices]; zs=[v.co.z for v in mesh.vertices]
xmin,xmax=min(xs),max(xs); zmin,zmax=min(zs),max(zs)
def ss(a,b,x):
    t=max(0.0,min(1.0,(x-a)/(b-a) if b!=a else 0.0)); return t*t*(3-2*t)
vg=boot.vertex_groups.get('gradient')
if vg is None: vg=boot.vertex_groups.new(name='gradient')
for v in mesh.vertices:
    fx=(v.co.x-xmin)/(xmax-xmin) if xmax!=xmin else 0
    fz=(v.co.z-zmin)/(zmax-zmin) if zmax!=zmin else 0
    base=ss(0.45,0.92,fx); dorsal=0.3+0.7*ss(0.30,0.90,fz)
    w=max(0.0,min(1.0,0.15+(1-0.15)*base*dorsal))
    vg.add([v.index], w, 'REPLACE')
print('gradient recomputed on', len(mesh.vertices), 'verts')
print('V rotated %.1f deg CW (top-down): tip -> +Y, ankle -> -Y' % abs(ROT_DEG))
