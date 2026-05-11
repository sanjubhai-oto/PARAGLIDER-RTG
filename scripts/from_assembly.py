"""Export each top-level shape from assembly.step keeping its assembly world coords.
Subshape mapping (by bbox/face count):
  0,1: fuselage main + inner -> merge into fuselage_asm.stl
  2,3: small mounts -> include in fuselage
  4: canopy (huge, at Z~1500mm)
  5: propeller (thin disc, Y~140mm behind body)
  6: left arm
  7: right arm
"""
import os, Part, MeshPart

ASM = "/Users/sanju/Downloads/paraglider step/assembly.step"
OUT = "/Users/sanju/paraglider_sim/meshes"

groups = {
    "fuselage_asm.stl": [0, 1, 2, 3],
    "canopy_asm.stl":   [4],
    "propeller_asm.stl":[5],
    "arm_left_asm.stl": [6],
    "arm_right_asm.stl":[7],
}

master = Part.Shape(); master.read(ASM)
subs = master.SubShapes

for name, idxs in groups.items():
    if len(idxs) == 1:
        shape = subs[idxs[0]]
    else:
        shape = Part.Compound([subs[i] for i in idxs])
    mesh = MeshPart.meshFromShape(Shape=shape, LinearDeflection=0.5,
                                   AngularDeflection=0.349, Relative=False)
    out = os.path.join(OUT, name)
    mesh.write(out)
    bb = shape.BoundBox
    print(f"{name}: facets={mesh.CountFacets} center=({bb.Center.x:.0f},{bb.Center.y:.0f},{bb.Center.z:.0f}) mm")
print("DONE")
