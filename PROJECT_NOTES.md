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
    ├── build_rims.py               # STEP 3: (beads REMOVED — cleanup only)
    ├── build_v_band.py             # STEP 3b: V-trim band (same tech as reinforce bands; replaces beads)
    ├── build_tongue.py             # STEP 4: thatched tongue + double rim + rotation
    ├── build_ankle_reinforce.py    # STEP 5: two solid ankle bands (lace/clip anchors)
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
| `Ankle_Reinforce` | MESH | Two solid bands on the outer layer (cuff + above-foot) for lace/clip anchors. Raycast parametric grid + Solidify. |

> The boot mesh still has a `GradientSolidify` modifier, but the solid shell is **not part of the final print** — the lattice handles everything (including the toe, via rib fusion). The export excludes the boot mesh entirely. The modifier's `gradient` vertex group is still used by `build_lattice.py` for toe rib thickening.

---

## Build Pipeline

Run scripts in this exact order. Each must execute in an **isolated namespace** (`exec(compile(open(path).read(), path, 'exec'), {})`) because they share variable names that collide.

```python
import bpy
base = '/home/dustin/Documents/Models/Roxanne Shoes/scripts/'
for fn in ['build_lattice.py', 'cut_v_through_lattice.py', 'build_rims.py', 'build_ankle_reinforce.py', 'build_v_band.py', 'build_tongue.py']:
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

### Step 3 — `build_rims.py` (beads REMOVED) + `build_v_band.py` (V trim)
- The old round rim **beads are gone**. They looked awkward once the solid reinforce bands existed, and the big outer bead fouled the latch/lace.
- **`build_v_band.py`** replaces them with a flat **V-trim band** along both V edges, built with the SAME technique as the reinforce bands:
  - **Built on the pristine shell via raycast** (same surface source as the reinforce bands); wall **1.5mm along the true normal** = SAME THICKNESS as the reinforce bands. Follows the surface (no bending up at the toe).
  - **Seamless merge**: the TOP is built manually (= base + 1.5mm·normal) rather than via Solidify, so where the V band overlaps a reinforce band, each top vert is **SNAPPED onto the reinforce band's evaluated top surface** (raycast the band mesh) → one continuous top surface, no step. (Solidify alone protruded ~0.5mm: the V band sits on the flat dorsal where the normal is vertical so 1.5mm wall = 1.5mm Z-rise, while the reinforce band wraps the curved foot where normals tilt so the same wall = only ~1.0–1.26mm Z-rise. Snapping eliminates the difference exactly.)
  - Footprint placed **explicitly in XY** (straight-down raycast) so the inner edge lands precisely at `edge − W_IN`, guaranteeing the cut lattice tube ends (measured ~0.8mm intrusion) are covered on BOTH sides (verified: gap clean with tongue hidden).
  - **Constant width** (W_IN 1.2mm into the gap + W_OUT 2.0mm over the lattice ≈ 1/4 of the reinforce bands), miters to a clean POINT at the V tip.
  - **Smooth outer edge**: the outer edge is a constant parallel offset of the V edge (never shaved/moved inward); the point comes from a miter at the tip, narrowing from the inside.
  - **REQUIRES `build_ankle_reinforce.py` to run first** (snaps to its evaluated mesh). Pipeline order reflects this.
  - Ankle end capped flat at the **print plane** (x=−9.9mm).
- The **ankle perimeter** needs no bead — the cuff reinforce band covers it.

### Step 4 — `build_tongue.py`
- Parametric curved surface (conforms to dorsal at tip, arcs toward centerline).
- Thatched crosshatch: two diagonal families in (u,v) space, clipped to a **superellipse boundary** so lines terminate into the rim (rectangles leave overflow at corners).
- **Double rim**: outer + inner, both smooth closed superellipse loops. Sides parallel at v=±0.90.
- **Visor bend**: `dome_factor` constant 1.0 (gentle even arc).
- **Rotation −4°** (pivot near tip): back dives into cavity, tip lifts into wall for attachment.

### Step 5 — `build_ankle_reinforce.py`
- Two **solid bands** on the outer layer for lace/clip anchor points: a cuff band (x[−9.8..3.2]mm) and an above-foot band (x[23.2..36.8]mm), lattice left open between them.
- Built as a **clean parametric grid** (NOT extracted mesh faces — those spike): for each x-station, raycast a dense angular fan from the smoothed centerline against the pristine closed shell (true surface points/normals), drop the dorsal hits inside the V polygon (keeps the tongue opening clear), resample every cross-section to a fixed point count by arc length → identical-length rows → clean quads, zero raggedness.
- Solidify outward (**1.5mm wall** — the user's set value; do NOT thicken. The V band carries the gradient thickening instead) caps + embeds the outer lattice tubes into a solid band.
- This raycast-grid technique is the one that worked; the earlier extract-faces-and-offset approach produced spikes and was scrapped.
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
ANKLE_X=-0.0115; TIP_X=0.1014; HALF_W_MAX=0.006  # V: 12mm wide fat end; tip +23.4mm (~26% of wedge) toward toe
ROT_DEG=-1.0    # V rotation (CW top-down), free knob
```
`TIP_X` history: 0.078 → 0.0914 (test print #1: opening too small) → **0.1014**
(+10mm more toward the toe, per request to lengthen the V cutout). The wedge
length is 89.5mm, so +23.4mm = ~26%. Toe mesh extends to x=0.1204, so there is
still ~19mm of solid fused toe material beyond the new tip.

### `build_tongue.py`
Shares `ANKLE_X`/`TIP_X`/`HALF_W_MAX`/`ROT_DEG` with the V cut (must stay in
sync). The tongue WIDTH and conform zone scale off `[ANKLE_X, TIP_X]`, but the
**tip position `FRONT_X` is PINNED** (0.0974) so deepening the V no longer drags
the tongue forward — the tongue stays short and retracts away from the toe.
At x=0.0974 the V is only ~0.4mm wide, so the 12.7mm-wide tip still embeds ~12mm
into the solid toe wall on both sides (hinge/attachment preserved).
```
FRONT_X=0.0974        # tongue tip PINNED (was TIP_X+0.006; decoupled from V depth)
BACK_X =-0.0105       # flush with ankle opening (x=-9.9mm vs opening -10.0mm)
THICKNESS=0.0010; HATCH_SPACING=0.0045; RIB_RADIUS=0.00055
SUP_N=2.5             # superellipse corner rounding (rim + lattice clip MUST match)
dome_factor=1.0       # visor bend (constant)
ROT_ANGLE_DEG=-5.0    # tongue pitch (pivot at FRONT_X-0.015); backed off 1deg from -6 after elongation poked tip ~0.05mm through dorsal
```

---

## Export — USE `safe_export_stl.py` ONLY

```python
exec(compile(open(base+'safe_export_stl.py').read(), base+'safe_export_stl.py', 'exec'), {})
```

Non-destructive: duplicates the 4 curve objects → converts COPIES to mesh → joins → **flattens the ankle (−X) end** (see below) → exports STL → deletes copies. Original curves untouched. The boot reference mesh is excluded (not printed).

**Ankle flatten (the PRINT SURFACE — ankle-down):** the boot prints **ankle-down**, so the −X end is the first layer on the build plate and MUST be perfectly flat. The tongue's **back tip** (center of the ankle hole) is the main contact reference. Everything that reaches the ankle — tongue back tip, cuff reinforce band edge, V-trim ankle end, ankle rim — is made **coplanar at the print plane** `ANKLE_FLAT_X = -0.0099` (−9.9mm; the pristine mesh is open at −10.0 = no surface, so −9.9 is the furthest reachable). At export, the joined copy is sliced by `bisect_plane` at ANKLE_FLAT_X; everything with `x < ANKLE_FLAT_X` is deleted (trims the Solidify flare of the cuff band flat) so every remaining cut face lies in that one plane. **Capping is DISABLED** (`CAP_MAX_AREA_M2 = 0`): the cut already makes all faces coplanar (flat first layer; lattice tube ends print as open rings, which is planar and doesn't rock). Earlier selective capping (`<30mm²`) created stray triangles at the ankle where the V-trim + tongue + band cut loops overlapped — disabling it fixed the garbage. The macro foot-opening loop is always skipped, so the ankle hole can **never** be sealed. If a fully solid flat first layer is ever needed, add a thin outward brim flange.

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
- V tongue cutout (150% wide, rotated, clean bisect edges; deepened TIP_X=0.1014)
- Tongue (thatched crosshatch, double rounded rim, visor bend, −4° rotation, tip PINNED at FRONT_X=0.0974 so it retracts from the deepened V; back tip at the print plane)
- **V-trim band** (`build_v_band.py`): flat trim along both V edges, SAME 1.5mm wall as the reinforce bands, follows the surface (no toe bend), constant width miters to a clean point (smooth outer edge, narrows from inside), covers cut lattice ends on both sides, and **snaps to the reinforce bands' tops to merge seamlessly** where they cross
- **Reinforce bands** raised to 1.8mm wall (clears the lattice; was 1.5mm)
- Sole-touch guarantee (offset 0.448mm → all crossings melt ≥0.1mm)
- Non-destructive STL export; **print plane** at x=−9.9mm (ankle-down, flat first layer; capping disabled → no ankle garbage)

**Known / accepted:**
- Faint seam at y≈−1.5 down the top center
- Test print #1 (TPU): structurally sound, but the V opening was too small to
  stuff the foot through comfortably → V wedge deepened
  (TIP_X 0.078 → 0.0914 → **0.1014**, now +26% / +10mm beyond original) in
  `cut_v_through_lattice.py` / `build_ankle_reinforce.py`. The **tongue is NOT
  lengthened** with it — `FRONT_X` pinned at 0.0974 so the tongue retracts back
  from the toe (10mm shorter than it would be if it tracked the V).
  Awaiting test print #2.

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
