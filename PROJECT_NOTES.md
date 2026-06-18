# Roxanne Shoes — 3D-Printed Dog Boot

A 3D-printable dog boot (TPU, FDM) for a dog in a wheelchair who drags/knuckles her feet. The design puts the wear/thickness on the **dorsal (top) surface** over the toe knuckles rather than on the sole. The boot is a breathable **bi-helical lattice shell** with a hinged tongue flap for paw entry.

---

## Coordinate System

The source mesh `left boot cutout meters` (units: meters, scale 1.0), ~13 × 6 × 5 cm:

| Axis | Meaning |
|---|---|
| +X | Toward toes (front). −X = ankle/cuff opening. |
| +Z | Up (dorsal/top). −Z = sole/bottom (paw pad). |
| +Y / −Y | Sides. |

The mesh is a 3D closed shell open only at the ankle cuff (−X) and the V tongue cutout (top). ~29k verts uncut.

---

## Files

```
Roxanne Shoes/
├── shoe.blend              # LIVE editable file (curves + objects intact)
├── shoe_export.stl         # Print-ready export (pure lattice, no shell) ~46 MB
├── PROJECT_NOTES.md        # this file
└── scripts/
    ├── build_lattice.py            # STEP 1: lattice on uncut mesh
    ├── cut_v_through_lattice.py    # STEP 2: cut V through mesh + trim lattice curves
    ├── build_rims.py               # STEP 3: smooth boundary rim spines
    ├── build_tongue.py             # STEP 4: thatched tongue + double rim + rotation
    ├── safe_export_stl.py          # NON-DESTRUCTIVE export — USE THIS
    ├── export_stl.py               # ⚠️ DESTRUCTIVE (old) — do not use
    ├── cut_v_curved.py             # OLD V-cut — superseded by cut_v_through_lattice.py
    └── diagnostics/                # measurement scripts
```

---

## Objects in `shoe.blend`

| Object | Type | Purpose |
|---|---|---|
| `left boot cutout meters` | MESH | Reference surface the lattice drapes on. **NOT printed.** Hidden from view. |
| `left boot cutout BACKUP` | MESH | **Pristine uncut mesh** (28,982 verts). The sacred restore point — all build scripts regenerate from this. Never modify. |
| `Lattice_OUTER` | CURVE | Outer family of bi-helical lattice (beveled tubes), 49 splines |
| `Lattice_INNER` | CURVE | Inner family (offset inward), 50 splines |
| `Rims` | CURVE | Solid spine tracing the V+ankle boundary perimeter |
| `Tongue` | CURVE | Thatched crosshatch flap + double rounded rim, 54 splines |

> The boot mesh still has a `GradientSolidify` modifier, but the solid shell is **not part of the final print** — the lattice handles everything (including the toe, via rib fusion). The export excludes the boot mesh entirely. The modifier's `gradient` vertex group is still used by `build_lattice.py` for toe rib thickening.

---

## Build Pipeline

Run scripts in this exact order. Each must execute in an **isolated namespace** (`exec(compile(open(path).read(), path, 'exec'), {})`) because they share variable names that collide.

```python
import bpy
base = '/home/dustin/Documents/Models/Roxanne Shoes/scripts/'
for fn in ['build_lattice.py', 'cut_v_through_lattice.py', 'build_rims.py', 'build_tongue.py']:
    exec(compile(open(base + fn).read(), base + fn, 'exec'), {})
bpy.data.objects['left boot cutout meters'].hide_set(True)
```

### Step 1 — `build_lattice.py`
- Restores the uncut mesh from BACKUP (apex intact → clean unwrap).
- Computes a **smoothed centerline** (per-X-slice centroid, heavy box-blur: 25 passes × half-width 4). Without this, lattice lines develop S-curves where the raw centroid jumps between slices.
- Girth angle `phi` (seam at top apex), globally unwrapped via BFS.
- **Per-triangle local unwrap** (lines ~112-116): the global unwrap has an unavoidable branch cut (~303 tears — topologically required, like combing a hairy sphere). Within each triangle, `pb,pc` are re-brought within `P/2` of `pa` before marching contours, so lines cross the branch cut continuously (0 apex terminations).
- Generates two contour families: `f = P·phi_lift ± T·s_norm`, marched per-triangle, chained with **tolerant KDTree matching** (tol=6µm). Round-key hashing fragments lines (94% shards); KDTree is required.
- Offsets each family along the surface normal: outer `+OFFSET_OUT`, inner `−OFFSET_IN` → two parallel layers that cross and fuse at crossings.
- Per-point rib radius scales with gradient weight (`TOE_SCALE`) → toe ribs thicker and fuse into a solid block; ankle ribs thin (breathable).

### Step 2 — `cut_v_through_lattice.py`
- Cuts the V tongue opening using **bisect-planes** (clean edges). Booleans on this open shell fail (they cut the sole instead of the top).
- Trims the already-built lattice curves that cross the V polygon. **Critical:** only trims points where `z > 0` (dorsal) — a point-in-polygon test alone wrongly removes sole lattice beneath the V.

### Step 3 — `build_rims.py`
- Traces open boundary edges (V + ankle perimeter) into a solid spine tube.
- **Direction-aware chaining** (continue straightest at junctions) + Laplacian smoothing. Naive chaining causes direction reversals → jagged zigzag tube.

### Step 4 — `build_tongue.py`
- Parametric curved surface (conforms to dorsal at tip, arcs toward centerline).
- Thatched crosshatch: two diagonal families in (u,v) space, clipped to a **superellipse boundary** so lines terminate into the rim (rectangles leave overflow at corners).
- **Double rim**: outer + inner, both smooth closed superellipse loops. Sides parallel at v=±0.90.
- **Visor bend**: `dome_factor` constant 1.0 (gentle even arc).
- **Rotation −6°** (pivot near tip): back dives into cavity, tip lifts into wall for attachment.
- `use_fill_caps=False` — caps create blobs that look like stray ribs at rim junctions.
- No separate hinge object — the tongue's tip conforming into the lattice wall is the attachment.

---

## The Seam (KNOWN, ACCEPTED)

There is a faint lattice density line ("seam") running down the top center at roughly y ≈ −1.5mm. Contour lines bunch where the girth angle has a shallow gradient across the broad flat dorsal top — more levels crowd into a thin band. The per-triangle unwrap fixed apex *breaks* but did not eliminate this bunching, which is inherent to using an angular coordinate on a nearly-flat surface.

Approaches that did not work:
- Surface-distance (geodesic) coordinate `u` — made it worse.
- Building on uncut mesh to "heal the tear" — the tear is topological, not from the V cut.
- Bridging / tangent extension / clockwise curling toward partner ends — all produced spaghetti.

The current version is accepted. Leave it unless explicitly asked to revisit.

---

## Parameters (Tunable Knobs)

### `build_lattice.py` (~line 100)
```
N_LINES   = 56        # rib count per family
SWEEP     = 0.50      # spiral twist (0=rings, 1=full turn)
RIB_RADIUS= 0.00051   # base rib radius (0.51mm)
OFFSET_OUT= 0.000448  # outer layer offset
OFFSET_IN = 0.000448  # inner layer offset
SMOOTH_ITERS = 6      # Laplacian smoothing passes
TOE_SCALE = 2.55      # rib radius multiplier at toe (>1.7 fuses crossings)
```
OFFSET = 0.000448 was computed to guarantee ≥0.1mm melt at every sole crossing. Don't change without re-running `diagnostics/sole_touch.py`.

### `cut_v_through_lattice.py` (~line 42)
```
ANKLE_X=-0.0115; TIP_X=0.078; HALF_W_MAX=0.006  # V: 12mm wide fat end
ROT_DEG=-1.0    # V rotation (CW top-down), free knob
```

### `build_tongue.py`
```
FRONT_X=TIP_X+0.006   # tongue tip embed (hinge/attachment)
BACK_X =-0.0105       # flush with ankle opening (x=-9.9mm vs opening -10.0mm)
THICKNESS=0.0010; HATCH_SPACING=0.0045; RIB_RADIUS=0.00055
SUP_N=2.5             # superellipse corner rounding (rim + lattice clip MUST match)
dome_factor=1.0       # visor bend (constant)
ROT_ANGLE_DEG=-6.0    # tongue pitch (pivot at FRONT_X-0.015)
```

---

## Export — USE `safe_export_stl.py` ONLY

```python
exec(compile(open(base+'safe_export_stl.py').read(), base+'safe_export_stl.py', 'exec'), {})
```

Non-destructive: duplicates the 4 curve objects → converts COPIES to mesh → joins → exports STL → deletes copies. Original curves untouched. The boot reference mesh is excluded (not printed).

**Never use `export_stl.py`** — it converts/joins the originals in-place, destroying the editable curves.

The exported STL contains Lattice_OUTER + Lattice_INNER + Rims + Tongue, joined (~485k verts). Overlapping geometry is left as-is — the slicer merges it during slicing (standard for multi-part lattice prints).

---

## Critical Gotchas

1. **Solidify `thickness_vertex_group` is the FLOOR**, not influence. Setting it to 1.0 = uniform thickness everywhere (defeats the gradient). It means "thickness at zero vertex-group weight."

2. **Measure the EVALUATED mesh, not the weight group.** Reading weights and reporting them as "thickness" is circular and cannot detect the floor bug. Build a KDTree of base verts, find nearest per evaluated vert, compute displacement along the base normal.

3. **Booleans on this open shell fail** (cut the sole instead of the top). Use bisect-planes for cuts.

4. **Chaining requires tolerance.** Round-key (5-decimal) hashing fragments contour lines; points 1µm apart straddle buckets. Use KDTree with tol.

5. **`sample()` with out-of-bounds params wraps silently** → garbage points. The tongue superellipse must keep u in [0,1] (centered at u=0.5, radius 0.5), not −1..+1.

6. **`use_fill_caps`** on lattice curves creates solid hemispheres that stack into blobs near rims. Tongue uses `False`.

7. **Multiple `exec()` calls need isolated namespaces** — scripts share names like `d`, `base`. Use `exec(compile(...), {})` per script.

8. **The rotation knob (`ROT_DEG`) was historically coupled to seam visibility** (only certain values hid the seam). After switching to build-lattice-on-uncut-mesh-first, it's freer, but re-check the seam metric after changing it.

9. **BACKUP + build scripts are the recovery path.** If `shoe.blend` ever loses its curves, re-running the 4-step pipeline regenerates everything from the pristine BACKUP mesh.

---

## Status

**Complete:**
- Bi-helical contour lattice (per-triangle unwrap, smooth, continuous, fuses at toe)
- V tongue cutout (150% wide, rotated, clean bisect edges)
- Smooth rim spines
- Tongue (thatched crosshatch, double rounded rim, visor bend, −6° rotation, flush with ankle)
- Sole-touch guarantee (offset 0.448mm → all crossings melt ≥0.1mm)
- Non-destructive STL export

**Known / accepted:**
- Faint seam at y≈−1.5 down the top center
- Printability untested (STL exported, not yet test-printed)

**Not done:**
- Clasp / lace / strap system
- Right boot (only left exists; needs a mirror)
- Test print review (TPU lattice overhangs may sag)

---

## Iterating

1. Edit a build script (knob values) or tweak objects directly in Blender.
2. Run the pipeline (above) — idempotent, regenerates from BACKUP.
3. Save: `bpy.ops.wm.save_as_mainfile(filepath='/home/dustin/Documents/Models/Roxanne Shoes/shoe.blend')`
4. Export: run `safe_export_stl.py`.
