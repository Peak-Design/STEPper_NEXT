import sys, json, bpy
argv = sys.argv[sys.argv.index("--") + 1:]
blend_path, out_json = argv[0], argv[1]

# Replicate STEPPER_OT_background_import._append_result
pre_scenes = set(bpy.data.scenes)
with bpy.data.libraries.load(blend_path, link=False) as (df, dt):
    dt.scenes = list(df.scenes)
new_scenes = [s for s in bpy.data.scenes if s not in pre_scenes]
target = bpy.context.scene.collection
for ws in new_scenes:
    for col in list(ws.collection.children):
        ws.collection.children.unlink(col)
        target.children.link(col)
    for obj in list(ws.collection.objects):
        ws.collection.objects.unlink(obj)
        target.objects.link(obj)
    bpy.data.scenes.remove(ws)

bpy.context.view_layer.update()

objects = []
for obj in sorted(bpy.data.objects, key=lambda o: o.name):
    rec = {"name": obj.name, "type": obj.type,
           "parent": obj.parent.name if obj.parent else None,
           "matrix_world": [round(v, 6) for row in obj.matrix_world for v in row]}
    if obj.type == "MESH":
        rec.update(verts=len(obj.data.vertices), polys=len(obj.data.polygons),
                   loops=len(obj.data.loops),
                   materials=[{"name": m.name, "diffuse": [round(c,5) for c in m.diffuse_color]} for m in obj.data.materials if m])
    objects.append(rec)
json.dump({"objects": objects}, open(out_json, "w"), indent=1, sort_keys=True)
print(f"APPEND_OK objects={len(objects)}")
