"""Rim beads REMOVED. The V-edge beads are replaced by V_Band (build_v_band.py),
a smooth raised lip that defines the V opening and covers the cut lattice ends.
The ankle perimeter is covered by the cuff reinforce band (build_ankle_reinforce.py).
This script now just cleans up any old Rims/Rim objects from previous builds so
stale beads don't linger in the scene."""
import bpy
for nm in ('Rims', 'Rim'):
    old = bpy.data.objects.get(nm)
    if old:
        bpy.data.objects.remove(old, do_unlink=True)
        print('removed old', nm)
print('=== RIMS: beads removed (V edges -> V_Band; ankle -> cuff band) ===')
