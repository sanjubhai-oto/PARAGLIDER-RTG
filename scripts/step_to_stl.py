import bpy, sys, os

argv = sys.argv[sys.argv.index("--")+1:]
src, dst = argv[0], argv[1]

bpy.ops.wm.read_factory_settings(use_empty=True)

try:
    bpy.ops.wm.obj_import  # placeholder
except Exception:
    pass

ext = os.path.splitext(src)[1].lower()
if ext in (".step", ".stp"):
    try:
        bpy.ops.wm.step_import(filepath=src)
    except AttributeError:
        try:
            bpy.ops.import_mesh.step(filepath=src)
        except Exception as e:
            print(f"STEP import failed: {e}")
            sys.exit(2)
else:
    print(f"Unsupported ext: {ext}")
    sys.exit(2)

for obj in bpy.context.scene.objects:
    if obj.type == 'MESH':
        obj.select_set(True)

bpy.ops.wm.stl_export(filepath=dst, export_selected_objects=True, apply_modifiers=True, global_scale=0.001)
print(f"OK -> {dst}")
