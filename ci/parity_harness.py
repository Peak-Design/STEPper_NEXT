"""Headless parity harness for STEPper NEXT.

Usage:
  blender.exe -b --factory-startup --python test_import.py -- <file.step> <out.json>

Imports the STEP file with the STEPper_NEXT addon and dumps a deterministic
JSON snapshot of the resulting scene (objects, mesh counts, materials,
transforms) for baseline/parity diffing.
"""
import json
import os
import sys
import time

import addon_utils
import bpy

argv = sys.argv[sys.argv.index("--") + 1:]
step_file, out_json = argv[0], argv[1]

bpy.ops.preferences.addon_enable(module="STEPper_NEXT")

t0 = time.perf_counter()
result = bpy.ops.import_scene.occ_import_step(
    filepath=step_file, override_file=os.path.basename(step_file)
)
elapsed = time.perf_counter() - t0

objects = []
for obj in sorted(bpy.data.objects, key=lambda o: o.name):
    rec = {
        "name": obj.name,
        "type": obj.type,
        "parent": obj.parent.name if obj.parent else None,
        "matrix_world": [round(v, 6) for row in obj.matrix_world for v in row],
    }
    if obj.type == "MESH":
        me = obj.data
        rec["verts"] = len(me.vertices)
        rec["polys"] = len(me.polygons)
        rec["loops"] = len(me.loops)
        rec["materials"] = [
            {
                "name": m.name,
                "diffuse": [round(c, 5) for c in m.diffuse_color],
            }
            for m in me.materials
            if m is not None
        ]
    objects.append(rec)

snapshot = {
    "file": step_file,
    "operator_result": list(result),
    "import_seconds": round(elapsed, 3),
    "num_objects": len(objects),
    "objects": objects,
}

with open(out_json, "w") as f:
    json.dump(snapshot, f, indent=1, sort_keys=True)

print(f"SNAPSHOT_WRITTEN {out_json} objects={len(objects)} time={elapsed:.2f}s")
