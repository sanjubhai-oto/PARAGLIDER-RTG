import Part, FreeCAD
SRC = "/Users/sanju/Downloads/paraglider step"
files = [("BODY.step","body"), ("PROPELLER.step","prop"),
         ("left arm.step","larm"), ("right arm.step","rarm")]
for f, name in files:
    s = Part.Shape(); s.read(f"{SRC}/{f}")
    bb = s.BoundBox
    print(f"{name}: center({bb.Center.x:.1f},{bb.Center.y:.1f},{bb.Center.z:.1f})  "
          f"min({bb.XMin:.1f},{bb.YMin:.1f},{bb.ZMin:.1f})  "
          f"max({bb.XMax:.1f},{bb.YMax:.1f},{bb.ZMax:.1f})  mm")
