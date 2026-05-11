import sys, os
import FreeCAD, Part, Mesh, MeshPart

src, dst = sys.argv[1], sys.argv[2]
shape = Part.Shape()
shape.read(src)

# Tessellate. linear deflection in mm; smaller = finer mesh.
mesh = MeshPart.meshFromShape(Shape=shape, LinearDeflection=0.5, AngularDeflection=0.349, Relative=False)
mesh.write(dst)
print(f"OK {src} -> {dst}  faces={mesh.CountFacets}")
