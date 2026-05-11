import os, Part, MeshPart

SRC = "/Users/sanju/Downloads/paraglider step"
DST = "/Users/sanju/paraglider_sim/meshes"
os.makedirs(DST, exist_ok=True)

files = {
    "BODY.step": "fuselage.stl",
    "PROPELLER.step": "propeller.stl",
    "left arm.step": "arm_left.stl",
    "right arm.step": "arm_right.stl",
}

for src_name, dst_name in files.items():
    src = os.path.join(SRC, src_name)
    dst = os.path.join(DST, dst_name)
    print(f"-> {src_name}")
    shape = Part.Shape()
    shape.read(src)
    mesh = MeshPart.meshFromShape(Shape=shape, LinearDeflection=0.5, AngularDeflection=0.349, Relative=False)
    mesh.write(dst)
    bb = shape.BoundBox
    print(f"   {dst_name}  facets={mesh.CountFacets}  bbox(mm) X={bb.XLength:.1f} Y={bb.YLength:.1f} Z={bb.ZLength:.1f}")
print("DONE")
