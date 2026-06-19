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
    ├── build_rims.py               # STEP 3a: (beads REMOVED — cleanup only)
    ├── build_ankle_reinforce.py    # STEP 3b: two solid bands (MUST run before build_v_band)
    ├── build_v_band.py             # STEP 3c: V-trim band (snaps to the reinforce bands)
    ├── build_tongue.py             # STEP 4: thatched tongue + double rim + rotation
    ├── safe_export_stl.py          # NON-DESTRUCTIVE export (baseline, no flatten) — USE THIS
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
| `V_Band` | MESH | Flat V-trim band along both V edges (replaces the old Rims beads). Same technique as Ankle_Reinforce. |
| `Tongue` | CURVE | Thatched crosshatch flap + double rounded rim, 54 splines |
| `Ankle_Reinforce` | MESH | Two solid bands on the outer layer (cuff + above-foot) for lace/clip anchors. Raycast parametric grid + Solidify (spans both lattice layers). Built BEFORE V_Band (it's the snap target). |

> The boot mesh still has a `GradientSolidify` modifier, but the solid shell is **not part of the final print** — the lattice handles everything (including the toe, via rib fusion). The export excludes the boot mesh entirely. The modifier's `gradient` vertex group is still used by `build_lattice.py` for toe rib thickening.

---

## Build Pipeline

Run scripts in this exact order. Each must execute in an **isolated namespace** (`exec(compile(open(path).read(), path, 'exec'), {})`) because they share variable names that collide. **`build_ankle_reinforce.py` MUST run before `build_v_band.py`** (the V band snaps to the reinforce band's evaluated mesh).

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

### Step 3 — `build_rims.py` (beads REMOVED) + `build_v_band.py` (V trim, v6)
- The old round rim **beads are gone**. They looked awkward once the solid reinforce bands existed, and the big outer bead fouled the latch/lace.
- **`build_v_band.py`** replaces them with a flat **V-trim band** along both V edges, built with the SAME technique as the reinforce bands (raycast pristine shell + wall along the true normal). Wall `WALL=0.00165` (slightly thicker than the 1.5mm reinforce bands so their ends hide *under* the V band).
  - **Built on the pristine shell via centerline-relative raycast** (`surf()` = ray from smoothed centerline toward the dorsal) — SAME technique as the reinforce bands. Straight-down raycast hit the ankle SIDE WALL (z~12.7mm vs dorsal ~18mm) and produced a 5mm dead-end gap; centerline aim always hits the correct dorsal surface.
  - **TOP built manually** (= base + WALL·normal), NOT via Solidify. Two reasons Solidify was wrong here: (a) on the flat dorsal the normal is vertical so 1.5mm wall = 1.5mm Z-rise, while the reinforce band wraps the curved foot where normals tilt so the same wall = only ~1.0–1.26mm Z-rise → a ~0.3–0.5mm step; (b) Solidify flares open boundary edges backward at the ankle.
  - **Hides the reinforce-band ends**: in the overlap X-ranges, the top Z is `max(natural WALL height, reinforce_top + 0.3mm margin)` — the V band always sits at least 0.3mm ABOVE the band ends, so they tuck under it. ⚠ Earlier code SNAPPED the V band flush to the band top, which clamped the height regardless of `WALL` — changing the wall did nothing visible. `max()` not assignment.
  - **Two tip verts** (bottom on surface + top at surface+wall), not one. A single shared tip vert forced the top cap to dip to the surface Z → a sunken notch that read as a "dead-end then resume" gap. The tip Z is probed ROBUSTLY by averaging 3 points back from the exact edge (a single raycast at x=TIP_X hits low side geometry).
  - **Full-length taper with a floor**: `width_scale_at(x)` smoothsteps from full width (W_IN+W_OUT) down to `MIN_WIDTH=0.8mm` at the tip, starting past the foot band (x>40mm). The two arms still OVERLAP at the tip (don't vanish to a point). Earlier "collapse in last 12%" was too abrupt and read as blunt/rounded.
  - **`surf()` extrapolates past the open cuff** (x<−9.9, no surface) by reusing the nearest valid hit shifted in −X, so the rails extend to meet the cuff band at the ankle top without a jog. ⚠ Earlier extrapolation that shifted Y by the V-edge slope drifted rightward (the jog) — keep Y fixed, shift X only.
  - **Ankle end at x=−11.5mm** (meets the cuff band at the new print plane).
  - **REQUIRES `build_ankle_reinforce.py` to run first** (snaps to its evaluated mesh). Pipeline order reflects this.
- The **ankle perimeter** needs no bead — the cuff reinforce band covers it.

### Step 4 — `build_tongue.py`
- Parametric curved surface (conforms to dorsal at tip, arcs toward centerline).
- Thatched crosshatch: two diagonal families in (u,v) space, clipped to a **superellipse boundary** so lines terminate into the rim (rectangles leave overflow at corners).
- **Double rim**: outer + inner, both smooth closed superellipse loops. Sides parallel at v=±0.90.
- **Visor bend**: `dome_factor` constant 1.0 (gentle even arc).
- **Rotation −4°** (pivot near tip): back dives into cavity, tip lifts into wall for attachment.
- **`FRONT_X` pinned** (0.0974) so the front tip stays attached to the toe wall; length changes happen on the BACK side only (`BACK_X`). Moving the whole object in X detaches the front — never do that.
- **`BACK_X = −0.01066`** so the rotated back tip lands flush at the print plane x=−11.5mm.

### Step 5 — `build_ankle_reinforce.py`
- Two **solid bands** on the outer layer for lace/clip anchor points: a cuff band (x[−11.5..3.2]mm, **extended 1mm past the ankle rim** to cover the ankle top) and an above-foot band (x[23.2..36.8]mm), lattice left open between them.
- Built as a **clean parametric grid** (NOT extracted mesh faces — those spike): for each x-station, raycast a dense angular fan from the smoothed centerline against the pristine closed shell (true surface points/normals), drop the dorsal hits inside the V polygon (keeps the tongue opening clear), resample every cross-section to a fixed point count by arc length → identical-length rows → clean quads, zero raggedness.
- **Solidify spans BOTH lattice layers**: `thickness=0.0025, offset=0.20` → outer face +1.5mm (covers outer lattice, matches V band), inner face −1.0mm (covers the INNER lattice layer, into the cavity toward the ankle). Was `offset=1.0` (all outward), which left the inner lattice tube ends exposed at the ankle.
- **Open-cuff extrapolation**: past x≈−8.6mm the pristine mesh is OPEN (ankle cuff, no surface), so `cross_section_arc` returns None. Stations there reuse the nearest valid arc, shifted in −X (centroid-following) so the band extends smoothly over the ankle top. Without this the band dead-ends at −8.6mm.
- **Collar-to-rail gap fill**: the V is ASYMMETRIC at the ankle (left rail inner edge y=−3.77mm, right y=+5.29mm — V rotation + offset convergence skew them), so a symmetric margin can't close both gaps. Uses a shrunk V polygon (`EAR_MARGIN=0.0013`) so the collar wraps into the V gap to meet the rails. Tuned by iteration; going further makes corners poke out on both materials.
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
BACK_X =-0.01066      # extended toward ankle so ROTATED back tip lands at print plane x=-11.5mm. Length added on ankle side ONLY (front stays attached).
THICKNESS=0.0010; HATCH_SPACING=0.0045; RIB_RADIUS=0.00055
SUP_N=2.5             # superellipse corner rounding (rim + lattice clip MUST match)
dome_factor=1.0       # visor bend (constant)
ROT_ANGLE_DEG=-5.0    # tongue pitch (pivot at FRONT_X-0.015); backed off 1deg from -6 after elongation poked tip ~0.05mm through dorsal
```

### `build_v_band.py`
```
W_IN=0.0012; W_OUT=0.0020; WALL=0.00165   # wall slightly > reinforce bands (1.5) so their ends hide under it
MIN_WIDTH=0.0008; TAPER_START_X=0.040     # full width past the foot band, then smoothstep taper to 0.8mm floor at tip
ANKLE_END_X=-0.0115                       # meets the cuff band at the print plane
HIDE_MARGIN=0.0003                        # V band top sits this far ABOVE reinforce band ends (max(), not snap-to-flush)
```

### `build_ankle_reinforce.py`
```
WALL=0.00150            # 1.5mm (do NOT thicken; V band carries gradient thickening)
Solidify: thickness=0.00250, offset=0.20   # spans both lattice layers: outer +1.5mm, inner -1.0mm
STRIPS cuff=(-0.0115, 0.0032)  foot=(0.02325, 0.03675)   # cuff extended 1mm past ankle rim
EAR_MARGIN=0.0013       # collar wraps into V gap to meet rails; tuned by iteration (further = corners poke out)
```

---

## Export — USE `safe_export_stl.py` ONLY

```python
exec(compile(open(base+'safe_export_stl.py').read(), base+'safe_export_stl.py', 'exec'), {})
```

Non-destructive: duplicates the objects → converts COPIES to mesh → joins → exports STL → deletes copies. Originals untouched. The boot reference mesh is excluded (not printed).

**⚠ BASELINE EXPORT — NO ANKLE FLATTEN.** The export currently just joins + exports. There is **no bisect/cap pass**. This was removed after the bisect + `triangle_fill` capping produced a "camera-shutter" fan of stray triangles around the ankle hole (a billion random triangles jutting to connect wrong parts). The garbage was structural — it came from overlapping V-trim + tongue + band cut loops getting capped with `triangle_fill`. Removing it entirely gave a clean baseline.

**Consequence:** the ankle end is NOT bisect-flattened to a single plane. Objects extend to their natural extents: tongue back tip and cuff band reach x≈−11.5mm, V rails to −8.6mm, lattice tubes to −10.3mm. The intended print plane is x=−11.5mm (ankle-down), enforced by building the objects to terminate there, NOT by export cutting. If first-layer adhesion is poor because the ankle isn't perfectly planar, revisit by either (a) extending all ankle-terminating objects to exactly x=−11.5 (current state — close but lattice tubes are short), or (b) re-adding a clean flatten that does NOT use `triangle_fill` (e.g. only `bisect_plane` + `delete`, accepting open tube rings as the first layer). Do NOT re-add `triangle_fill` capping — it is the source of the shutter garbage.

**Never use `export_stl.py`** — it converts/joins the originals in-place, destroying the editable curves.

The exported STL contains Lattice_OUTER + Lattice_INNER + V_Band + Tongue + Ankle_Reinforce, joined (~477k verts). Overlapping geometry is left as-is — the slicer merges it during slicing (standard for multi-part lattice prints).

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

9. **BACKUP + build scripts are the recovery path.** If `shoe.blend` ever loses its curves, re-running the pipeline regenerates everything from the pristine BACKUP mesh.

10. **`triangle_fill` on cut edges = camera-shutter garbage.** Capping the ankle bisect cut with `triangle_fill` (even selectively, by loop area) produced a fan of stray triangles jutting around the ankle hole, connecting wrong parts. The cut loops from overlapping objects (V-trim + tongue + band) triangulate into a mess. **Do not re-add `triangle_fill` capping to the export.** If a flat ankle is needed, use `bisect_plane` + `delete` only and accept open tube rings as the first layer.

11. **Solidify flares open boundary edges backward.** At the open ankle cuff, the Solidify modifier extrudes the boundary edges along their −X-pointing normals, producing verts to x=−11.4mm (past the print plane) = the "random triangles jutting off." Either (a) apply + bisect-trim the modifier in-script, or (b) make sure the underlying surface extends past the plane so there are no open boundaries there.

12. **Snap-to-band-top defeats thickness changes.** If a band's top is snapped (assigned) to another band's top Z to "merge seamlessly," then changing the wall thickness does NOTHING at the overlap — the snap clamps it. To hide one band's ends under another, use `max(natural_height, other_top + margin)` so the thicker band always wins, not assignment.

13. **One shared tip vert = a sunken notch.** If both the bottom cap (on the surface) and the top cap (surface+wall) of a tapering band share a single convergence vertex, that vert can only be at one height — the top dips to the surface Z and reads as a "dead-end then resume" gap. Use TWO tip verts (bottom + top) and probe the tip Z by averaging several nearby points (a single raycast at the exact mesh edge hits low side geometry).

14. **Centerline-relative raycast, not straight-down.** Straight-down rays at the ankle hit the SIDE WALL / open cuff (z~12.7mm vs dorsal ~18mm) and produce stray verts + gaps. Ray from the smoothed centerline toward the dorsal (the reinforce-band technique) always hits the correct surface. This is non-negotiable for anything near the ankle.

15. **Extrapolate open-cuff stations by X-shift only, not Y.** Past x≈−8.6mm the mesh is open (no surface). Reusing a valid arc and shifting it to the missing station must shift in X only; shifting Y by the V-edge slope drifts sideways (a visible jog where both rails kick right).

16. **The V is ASYMMETRIC at the ankle.** V rotation + the offset convergence point skew the rails (left inner edge y=−3.77mm, right y=+5.29mm at x=−8.6mm). Symmetric polygon margins can't close a gap to both rails — use a shrunk V polygon (EAR_MARGIN) and/or per-side arc endpoint extension.

---

## Status

**Complete:**
- Bi-helical contour lattice (per-triangle unwrap, smooth, continuous, fuses at toe)
- V tongue cutout (150% wide, rotated, clean bisect edges; deepened TIP_X=0.1014)
- Tongue (thatched crosshatch, double rounded rim, visor bend, −4° rotation, FRONT_X pinned at 0.0974; BACK_X extended to −0.01066 so the back tip is flush at the print plane x=−11.5mm; front attachment preserved)
- **V-trim band v6** (`build_v_band.py`): flat trim along both V edges, 1.65mm wall (slightly > reinforce bands so their ends hide under it), centerline-raycast surface, manual top with `max()` hide-margin, two tip verts (no sunken notch), full-length taper with 0.8mm floor (arms overlap at tip), extrapolates past the open cuff to meet the collar
- **Reinforce bands**: 1.5mm wall, Solidify spans both lattice layers (outer +1.5mm / inner −1.0mm); cuff band extended 1mm past the ankle rim; collar-to-rail gap filled (EAR_MARGIN=1.3mm)
- Sole-touch guarantee (offset 0.448mm → all crossings melt ≥0.1mm)
- Non-destructive STL export (baseline — no ankle flatten/cap)

**Known / accepted:**
- Faint seam at y≈−1.5 down the top center
- **Ankle is NOT bisect-flattened.** Objects terminate near x=−11.5mm by construction (tongue back tip, cuff band, V rails), but lattice tubes still reach −10.3mm and there's no export cut. If test print #2 shows poor first-layer adhesion or rocking, this is the place to revisit (see Export section — do NOT re-add `triangle_fill`).
- Test print #1 (TPU): structurally sound, V opening too small → deepened (TIP_X 0.078 → 0.0914 → 0.1014). Tongue NOT lengthened with it (FRONT_X pinned). Awaiting test print #2.

**Not done:**
- Clasp / lace / strap system
- Right boot (only left exists; needs a mirror)
- Test print review (TPU lattice overhangs may sag)

---

## Working with the Blender MCP server (LIVE edits)

A Blender MCP server (`blender_execute_blender_code`) is available to edit the **live** running Blender instance — far better than background rebuilds because the user sees changes in their viewport without reopening the file. But it has sharp edges:

1. **Live updates don't always propagate to the viewport.** `exec()`-ing a build script changes the data, but the viewport can show a stale evaluated mesh. Force redraw every way: `view_layer.update()`, `depsgraph.update()`, `region_3d.update()`, `area.tag_redraw()`, and select+activate the changed object. If the user still sees no change, the nuclear option is: save, then have them `File → Revert`.
2. **Verify on disk, not via the live session.** The live session can report success while the saved file is stale (object not actually replaced, or save silently dropped the change). Ground truth = open the saved file in a FRESH background Blender (`blender --background --python verify.py`) and read the verts. Do this before claiming anything is done.
3. **Inline `exec` code must not end in `}`.** The MCP wraps code in JSON; a trailing `}` (e.g. from a one-liner `print('x')}`) causes `SyntaxError: unmatched '}'`. **Always write multi-line code to a `/tmp/*.py` file and `exec(open(...).read())` it** — never paste substantial code inline.
4. **`bpy.ops.wm.open_mainfile` inside MCP kills the screen context.** After reopening a file, `bpy.context.screen` becomes None, so viewport manipulation throws `'NoneType' object has no attribute 'areas'`. To show the user a saved state: reconnect the MCP, or use the MCP's own `blender_jump_to_view3d_object_by_name` tool to frame an object (don't hand-roll the view matrix post-reopen).
5. **`bpy.ops.render.opengl` crashes in `--background`** (no viewport). To render in background, create a Camera object and use `bpy.ops.render.render(write_still=True)` with EEVEE.
6. **Heavy single MCP calls can crash Blender** (connection reset). Run the pipeline one script per call, or write a `/tmp/*.py` orchestrator and `exec` that — don't cram lattice+cut+rims+vband+tongue+ankle into one inline block.
7. **The vision model (`zai/glm-4.6v` via `pi --model`) is unreliable for fine geometry** — it hallucinates reference lines, misreads 3/4 renders as "rounded" when the math shows a true point, and can't reliably tell a hole from lattice poking through. Use it only for coarse sanity checks; trust numerical vert/edge analysis over it, and **ask the user directly** for visual confirmation of anything subtle.

---

## Iterating

1. Edit a build script (knob values) or tweak objects directly in Blender.
2. Run the pipeline (above) — idempotent, regenerates from BACKUP. Order matters: `build_ankle_reinforce.py` before `build_v_band.py`.
3. For LIVE iteration: write the rebuild to `/tmp/*.py`, `exec` it via the MCP server, force redraw, save. Verify on disk with a background Blender probe.
4. Save: `bpy.ops.wm.save_as_mainfile(filepath='/home/dustin/Documents/Models/Roxanne Shoes/shoe.blend')`
5. Export: run `safe_export_stl.py`.
