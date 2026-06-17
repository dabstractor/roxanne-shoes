import bpy, bmesh, math, numpy as np

boot = bpy.data.objects['left boot cutout meters']
mesh = boot.data
V = np.array([tuple(v.co) for v in mesh.vertices], dtype=float)

xmin=float(V[:,0].min()); xmax=float(V[:,0].max())
NB=60
bed=np.linspace(xmin,xmax,NB+1)
bi=np.clip(np.digitize(V[:,0],bed)-1,0,NB-1)
cy=np.zeros(NB); cz=np.zeros(NB); cnt=np.zeros(NB)
for i in range(len(V)):
    cy[bi[i]]+=V[i,1]; cz[bi[i]]+=V[i,2]; cnt[bi[i]]+=1
cnt[cnt==0]=1; cy/=cnt; cz/=cnt
cy_v=cy[bi]; cz_v=cz[bi]
dy=V[:,1]-cy_v; dz=V[:,2]-cz_v
gamma=np.arctan2(dy, -dz)
phi=(math.pi - gamma)/(2.0*math.pi)
s_norm=(V[:,0]-xmin)/(xmax-xmin)

# --- neighbor-based phi inconsistency (wave detector) ---
adj=[[] for _ in range(len(V))]
for e in mesh.edges:
    a,b=e.vertices[0],e.vertices[1]
    adj[a].append(b); adj[b].append(a)

def circ_var(vals):
    # circular variance of phi (phi in [0,1)); 0=smooth, 1=chaotic
    cs=0.0; sn=0.0
    for v in vals:
        a=v*2*math.pi; cs+=math.cos(a); sn+=math.sin(a)
    n=len(vals) or 1
    return math.hypot(cs/n,sn/n)

incons=np.zeros(len(V))
for i in range(len(V)):
    nbrs=adj[i]
    if len(nbrs)<2: continue
    # compare neighbor phi to vertex phi, circular distance
    diffs=[]
    for j in nbrs:
        d=phi[j]-phi[i]
        # wrap to [-0.5,0.5]
        d=d-round(d)
        diffs.append(abs(d))
    incons[i]=max(diffs) if diffs else 0.0

# bin inconsistency by X position
print('=== WAVE DIAGNOSTIC: where is the phi field non-smooth? ===')
print('(high inconsistency = wavy ring at that length)')
xs_mm=V[:,0]*1000
for xb in range(NB):
    members=[i for i in range(len(V)) if bi[i]==xb]
    if not members: continue
    inc=[incons[i] for i in members]
    avg=sum(inc)/len(inc); mx=max(inc)
    frac=sum(1 for v in inc if v>0.15)/len(inc)
    cx=(bed[xb]+bed[xb+1])/2*1000
    flag = '  <<< WAVE' if (avg>0.08 or frac>0.3) else ''
    print('X %6.1fmm  verts=%4d  avgInc=%.3f maxInc=%.3f  fracBad=%.2f%s' % (cx, len(members), avg, mx, frac, flag))

# centerline kink check: 2nd derivative of centroid
print()
print('=== CENTERLINE kinks (2nd deriv of centroid y,z) ===')
acc=[]
for i in range(1,NB-1):
    ay=cy[i+1]-2*cy[i]+cy[i-1]
    az=cz[i+1]-2*cz[i]+cz[i-1]
    acc.append((math.hypot(ay,az)*1000, (bed[i]+bed[i+1])/2*1000, cy[i]*1000, cz[i]*1000))
acc.sort(reverse=True)
print('top 8 centroid-kink locations (X mm):')
for a,x,y,z in acc[:8]:
    print('  X %6.1fmm  kink=%.3fmm  (centroid y=%.1f z=%.1f)' % (x,a,y,z))
