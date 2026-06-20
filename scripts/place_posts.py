"""Place 4 Post.stl instances straddling the V on the two reinforce collars.

Spec (per user):
  - 2 posts per collar, straddling the V cutout (4 total).
  - Post near edge 2.25mm from the V cut edge, measured ALONG THE BAND SURFACE from
    the true V edge (raycast to the surface, NOT the collar-polygon arc endpoint).
    -> center 6.25mm from V edge. When the V zips shut: near edges 2.25+2.25=4.5mm
    gap, +8mm dia = 12.5mm on-center (latch spec).
  - Post AXIS = the V-RAIL radial direction (PE - C). Zero X-component; YZ angle =
    rail angle. The V closes by rotating each collar half about the longitudinal
    (X) axis to bring its rail to the top center (90deg); X-rotation preserves the
    X-component of any vector, so for the two posts to be PARALLEL after closure they
    must share an X-component (set to 0 here). A post whose axis starts at the rail
    angle rotates by the same closure angle and lands vertical -> both sides vertical
    = PARALLEL. (The earlier true-surface-normal axis had unequal nonzero X-components
    -> impossible to parallelize via an X-axis closure.)
  - Base inset 0.25mm INTO the band along the TRUE SURFACE NORMAL (pristine + 1.25mm;
    band outer face is pristine + 1.5mm via Solidify). The normal (not the axis) sets
    the inset depth so the post is rooted at the correct depth even though the axis is
    tilted ~20deg to the surface.
  - COST of the ~20deg rail-radial tilt (unavoidable on a curved collar): the base disk
    is tilted to the band -> away-from-V edge FLOATS ~1.1-1.5mm, toward-V edge buries.
    Fixed by extending the post bottom (see EXTENSION below), NOT by changing the axis.
  - +Z_LIFT (0.1mm world-Z): keeps the V-band material from protruding into the
    threaded top once the posts lean toward the V.
  - Threaded top side faces AWAY from the centerline (along +axis).
  - UPPER (cuff collar): 1/3 down cuff band x[-11.5..3.2]mm from -X -> x=-6.6mm.
  - LOWER (foot collar): dead center foot band x[23.25..36.75]mm -> x=30.0mm.

BOTTOM EXTENSION (user extends Post.stl bottom by a solid cylinder; thread TOP / zmax
unchanged, zmin lowered). EXTENSION_MM is AUTO-DETECTED = (zmax-zmin) - 4.5mm. The
placement anchors the ORIGINAL BASE (not the extension bottom) at the band surface, so
the extension reaches DOWN into the band and the tilted base disk makes full contact
everywhere (covers the ~1.5mm float). If Post.stl is NOT extended, EXTENSION=0 and the
placement reduces to the original behavior (+Z_LIFT only).

History of dead ends (do not reintroduce):
  - Walking from the collar cross_section_arc ENDPOINT (not the true V edge) ->
    near-edge landed ~3mm from V (collar polygon is shrunk EAR_MARGIN into the V).

Uses the SAME surface machinery as build_ankle_reinforce.py (centerline + V
polygon + cross-section raycast) so the posts sit on the same surface the bands
were built on. Posts are linked instances of a single Post_mesh data-block.
"""
import bpy, math, struct
import numpy as np
from mathutils import Vector, Matrix
from mathutils.bvhtree import BVHTree

PROJECT = '/home/dustin/Documents/Models/Roxanne Shoes/'
POST_STL = PROJECT + 'Post.stl'
boot = bpy.data.objects['left boot cutout meters']
src  = bpy.data.objects['left boot cutout BACKUP']; mesh = src.data
Mw = src.matrix_world
V = np.array([tuple(Mw @ v.co) for v in mesh.vertices], dtype=float)
polys=[list(p.vertices) for p in mesh.polygons]
bvh = BVHTree.FromPolygons([tuple(v) for v in V], polys, all_triangles=False)
xmin=float(V[:,0].min()); xmax=float(V[:,0].max())
NB=60; bed=np.linspace(xmin,xmax,NB+1); bi=np.clip(np.digitize(V[:,0],bed)-1,0,NB-1)
cy=np.zeros(NB); cz=np.zeros(NB); cnt=np.zeros(NB)
for i in range(len(V)): cy[bi[i]]+=V[i,1]; cz[bi[i]]+=V[i,2]; cnt[bi[i]]+=1
cnt[cnt==0]=1; cy/=cnt; cz/=cnt
def sm(a,p,h):
    a=a.copy()
    for _ in range(p):
        s=a.copy()
        for i in range(len(a)):
            lo=max(0,i-h);hi=min(len(a)-1,i+h);s[i]=a[lo:hi+1].mean()
        a=s
    return a
cy=sm(cy,25,4); cz=sm(cz,25,4); cxb=(bed[:-1]+bed[1:])/2
def cy_at(x):
    if x<=cxb[0]:return cy[0]
    if x>=cxb[-1]:return cy[-1]
    for i in range(NB-1):
        if cxb[i]<=x<=cxb[i+1]:
            t=(x-cxb[i])/(cxb[i+1]-cxb[i]); return cy[i]+(cy[i+1]-cy[i])*t
def cz_at(x):
    if x<=cxb[0]:return cz[0]
    if x>=cxb[-1]:return cz[-1]
    for i in range(NB-1):
        if cxb[i]<=x<=cxb[i+1]:
            t=(x-cxb[i])/(cxb[i+1]-cxb[i]); return cz[i]+(cz[i+1]-cz[i])*t
ANKLE_X=-0.0115;TIP_X=0.1014;HALF_W_MAX=0.006;DORSAL_Z=0.25;ROT_DEG=-1.0
th_v=math.radians(ROT_DEG);ct=math.cos(th_v);st=math.sin(th_v);cxv=(ANKLE_X+TIP_X)/2;cyv=0.0
def rot(x,y):
    dx=x-cxv;dy=y-cyv; return (cxv+dx*ct-dy*st, cyv+dx*st+dy*ct)
def v_width(x):
    s=(x-ANKLE_X)/(TIP_X-ANKLE_X); return HALF_W_MAX*max(0.0,1.0-s)
NS=41; sx=np.linspace(ANKLE_X,TIP_X,NS)
cyx=np.array([cy_at(x) for x in sx]); vwx=np.array([v_width(x) for x in sx])
lp=[rot(float(x),float(cyx[i]-vwx[i])) for i,x in enumerate(sx)]
rp=[rot(float(x),float(cyx[i]+vwx[i])) for i,x in enumerate(sx)]
EM=0.0013
lc=[rot(float(x),float(cyx[i]-vwx[i]+EM)) for i,x in enumerate(sx)]
rc=[rot(float(x),float(cyx[i]+vwx[i]-EM)) for i,x in enumerate(sx)]
polyc=lc+list(reversed(rc))
def inp(p,px,py):
    inside=False;n=len(p);j=n-1
    for i in range(n):
        xi,yi=p[i];xj,yj=p[j]
        if ((yi>py)!=(yj>py)) and (px<(xj-xi)*(py-yi)/((yj-yi) or 1e-12)+xi): inside=not inside
        j=i
    return inside
def arc(x):
    cyy=cy_at(x);czz=cz_at(x);o=(x,cyy,czz);hits=[]
    for k in range(144):
        th=2*math.pi*k/144
        loc,nrm,idx,d=bvh.ray_cast(o,(0,math.cos(th),math.sin(th)),0.06)
        if loc is None: continue
        if inp(polyc,loc.x,loc.y) and nrm.z>DORSAL_Z: continue
        hits.append((th,np.array([loc.x,loc.y,loc.z])))
    if len(hits)<8: return None
    hits.sort(key=lambda t:t[0]); n=len(hits); ths=[h[0] for h in hits]
    gaps=[(ths[(i+1)%n]-ths[i])%(2*math.pi) for i in range(n)]
    ig=max(range(n),key=lambda i:gaps[i]); order=[(ig+1+k)%n for k in range(n)]
    return [hits[i][1] for i in order]

left_world=np.array(lp); right_world=np.array(rp)
def true_v_edge(x, side):
    y_edge = np.interp(x, left_world[:,0], left_world[:,1]) if side=='L' \
             else np.interp(x, right_world[:,0], right_world[:,1])
    cyy=cy_at(x); czz=cz_at(x); C=np.array([x,cyy,czz])
    target=np.array([x, y_edge, czz+0.03]); d=target-C; d=d/np.linalg.norm(d)
    loc,nrm,idx,dist=bvh.ray_cast(tuple(C),tuple(d),0.06)
    if loc is None: return None, None
    return np.array([loc.x,loc.y,loc.z]), np.array([nrm.x,nrm.y,nrm.z])

SOLIDIFY_OUTER=0.00150; INSET=0.00025; POST_RADIUS=0.00400
CENTER_WALK=0.00225+POST_RADIUS

def walk_along(points, dist):
    pts=np.asarray(points); seg=np.linalg.norm(np.diff(pts,axis=0),axis=1)
    s=np.concatenate([[0.0],np.cumsum(seg)])
    if s[-1]<dist: P=pts[-1]
    else:
        j=int(np.searchsorted(s,dist)-1); j=max(0,min(j,len(seg)-1))
        t=(dist-s[j])/(seg[j] or 1e-12); P=pts[j]+t*(pts[j+1]-pts[j])
    return P

def angYZ(v): return math.degrees(math.atan2(v[2],v[1]))

def place_pair(x0, label):
    a=arc(float(x0))
    if a is None or len(a)<8:
        print('  !! %s: no arc'%label); return []
    cyy=cy_at(x0); czz=cz_at(x0); C=np.array([x0,cyy,czz])
    PEL,_=true_v_edge(x0,'L'); PER,_=true_v_edge(x0,'R')
    out=[]
    for side, PE, arcwalk in [('L',PEL,a),('R',PER,list(reversed(a)))]:
        poly_line=[PE]+arcwalk
        PC=walk_along(poly_line, CENTER_WALK)         # post center on surface, 2.25mm near-edge to V (UNCHANGED)
        # Surface normal at PC -> used ONLY for the base inset depth (0.25mm into the band
        # along the true normal, so the post is rooted at the correct depth).
        ddir=PC-C; ddir=ddir/(np.linalg.norm(ddir) or 1.0)
        loc2,nrm2,_,_=bvh.ray_cast(tuple(C),tuple(ddir),0.06)
        N_normal=np.array([nrm2.x,nrm2.y,nrm2.z]); N_normal=N_normal/(np.linalg.norm(N_normal) or 1.0)
        if np.dot(N_normal, PC-C) < 0: N_normal=-N_normal
        base=PC+(SOLIDIFY_OUTER-INSET)*N_normal      # inset 0.25mm into band along surface normal
        # AXIS = RAIL RADIAL (PE - C). Zero X-component; YZ angle = rail angle.
        # Why: the V closes by rotating each collar half about the longitudinal (X) axis to
        # bring its rail to the top center (90deg). X-rotation preserves the X-component of
        # any vector, so for the two posts to be PARALLEL after closure they MUST share an
        # X-component (set to 0 here). A post whose axis starts at the rail angle rotates by
        # the same closure angle alpha=90-rail_angle and lands at 90deg (vertical) -> both
        # sides vertical = PARALLEL. (The previous true-surface-normal axis had unequal,
        # nonzero X-components -> impossible to parallelize via an X-axis closure.)
        NR=(PE-C); NR=NR/(np.linalg.norm(NR) or 1.0)
        near=walk_along(poly_line, CENTER_WALK-POST_RADIUS); d_near=np.linalg.norm(near-PE)
        # verify closed-state parallelism: apply Rx(alpha) to NR, expect (0,0,1)
        theta_rail=math.atan2(PE[2]-czz, PE[1]-cyy); alpha=math.radians(90.0)-theta_rail
        yr=NR[1]*math.cos(alpha)-NR[2]*math.sin(alpha); zr=NR[1]*math.sin(alpha)+NR[2]*math.cos(alpha)
        closed=(NR[0],yr,zr)
        ok = abs(closed[2]-1.0)<0.02 and abs(closed[0])<0.02 and abs(closed[1])<0.02
        print('  %s_%s x=%5.1fmm | railYZ=%5.1f  closure a=%+5.1f  axis(rail-rad)YZ=%5.1f  -> CLOSED=(%+.3f,%+.3f,%+.3f) %s | near-edge=%4.2fmm'%(
            label,side,x0*1000, math.degrees(theta_rail), math.degrees(alpha), angYZ(NR),
            closed[0],closed[1],closed[2], 'PARALLEL' if ok else 'CHECK', d_near*1000))
        print('       center=(%6.2f,%6.2f,%5.2f)  axis=(%+.3f,%+.3f,%+.3f)  base=(%6.2f,%6.2f,%5.2f)'%(
            PC[0]*1000,PC[1]*1000,PC[2]*1000, NR[0],NR[1],NR[2], base[0]*1000,base[1]*1000,base[2]*1000))
        out.append((base,NR,label+'_'+side))
    return out

print('=== POST PLACE (axis = rail-radial -> PARALLEL when V closes; position unchanged) ===')
placements=[]
placements+=place_pair(-0.0066,'Cuff_Upper')
placements+=place_pair( 0.0300,'Foot_Lower')

# ---------- build post mesh + 4 linked instances ----------
with open(POST_STL,'rb') as f:
    f.read(80); ntri=struct.unpack('<I',f.read(4))[0]; verts_mm=[]; faces=[]
    for _ in range(ntri):
        f.read(12); i0=len(verts_mm)
        verts_mm.append(struct.unpack('<fff',f.read(12)))
        verts_mm.append(struct.unpack('<fff',f.read(12)))
        verts_mm.append(struct.unpack('<fff',f.read(12)))
        faces.append((i0,i0+1,i0+2)); f.read(2)
verts_mm=np.array(verts_mm,dtype=float)
zmin=verts_mm[:,2].min(); zmax=verts_mm[:,2].max()
# --- BOTTOM EXTENSION handling ---
# The original Post.stl is a 4.5mm-tall threaded cylinder. The user extends the BOTTOM by a
# solid cylinder (keeps the thread TOP / zmax unchanged, lowers zmin). We must NOT key the
# placement off zmin (that would float the whole post up by the extension length). Instead:
# detect the added length, anchor on the ORIGINAL base (zmin+EXTENSION), and let the extension
# reach DOWN into the band so the tilted base disk still makes full contact everywhere.
ORIGINAL_HEIGHT_MM = 4.5
EXTENSION_MM = max(0.0, (zmax - zmin) - ORIGINAL_HEIGHT_MM)
bc=verts_mm[verts_mm[:,2]<(zmin+0.05)]; bcx=bc[:,0].mean(); bcy=bc[:,1].mean()
for nm in list(bpy.data.objects.keys()):
    if nm.startswith('Post_'): bpy.data.objects.remove(bpy.data.objects[nm],do_unlink=True)
for mn in list(bpy.data.meshes.keys()):
    if mn.startswith('Post_') or mn=='Post_mesh': bpy.data.meshes.remove(bpy.data.meshes[mn])
pmesh=bpy.data.meshes.new('Post_mesh')
pmesh.from_pydata([tuple(v) for v in verts_mm],[],[tuple(f) for f in faces]); pmesh.update()
M_pre=(Matrix.Translation((-bcx*0.001,-bcy*0.001,-zmin*0.001)) @ Matrix.Scale(0.001,4))
Z_LIFT = 0.0005   # +0.5mm world-Z lift (running tally: +0.1 base, +0.5 test/visible, settled +0.3, +0.1 = 0.5 total; clears V-band/collar from thread holes)
print('Post mesh: zmin=%.3f zmax=%.3f  detected EXTENSION=%+.3fmm  Z_LIFT=%+.3fmm'%(
    zmin,zmax,EXTENSION_MM,Z_LIFT*1000))
def make_post(base_pos,normal,name):
    q=Vector((0,0,1)).rotation_difference(Vector(tuple(normal)))
    R=q.to_matrix().to_4x4()
    # M_pre anchors local-z=zmin -> 0. The ORIGINAL BASE sits at local-z=EXTENSION_MM, which in
    # world is translation + EXTENSION_MM*axis. To land the ORIGINAL BASE at base_pos (so the
    # extension extends DOWN into the band from there), shift translation by -EXTENSION*axis.
    # Then add the +Z_LIFT in world Z (keeps threads clear of the V-band).
    e=EXTENSION_MM*0.001
    shift=Vector((-e*normal[0], -e*normal[1], -e*normal[2]))
    T=Matrix.Translation(Vector(tuple(base_pos)) + shift + Vector((0.0,0.0,Z_LIFT)))
    obj=bpy.data.objects.new(name,pmesh)
    bpy.context.collection.objects.link(obj); obj.matrix_world=T@R@M_pre; obj.parent=boot
    return obj
print('\n=== POST OBJECTS ===')
for base_pos,normal,name in placements:
    o=make_post(base_pos,normal,name)
    print('  %-16s base=(%.2f,%.2f,%.2f)mm N=(%+.3f,%+.3f,%+.3f)'%(
        o.name,base_pos[0]*1000,base_pos[1]*1000,base_pos[2]*1000,normal[0],normal[1],normal[2]))
bpy.context.view_layer.update()
print('\nDONE.')
