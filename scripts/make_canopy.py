"""Procedural paraglider canopy mesh.
Arc cross-section, swept along span. Outputs STL.
RC scale: span 1.6m, chord 0.45m, arc rise 0.25m.
"""
import math, struct, os

SPAN = 1.6
CHORD = 0.45
RISE = 0.25
NSPAN = 32
NCHORD = 12

OUT_DIR = "/Users/sanju/paraglider_sim/meshes"
os.makedirs(OUT_DIR, exist_ok=True)

def arc_y(t):
    # t in [-1,1]. Positive rise so concave side faces DOWN (-Z) — real paraglider orientation.
    return RISE * (1 - t*t)

verts = []
for i in range(NSPAN+1):
    t = -1 + 2*i/NSPAN
    x = t * SPAN/2
    y_arc = arc_y(t)
    for j in range(NCHORD+1):
        s = j/NCHORD
        z_chord = (s - 0.5) * CHORD
        # Camber: simple parabolic, peak ~6% chord
        camber = 0.06 * CHORD * 4 * s * (1-s)
        verts.append((x, z_chord, y_arc + camber))

def idx(i,j): return i*(NCHORD+1) + j

tris = []
for i in range(NSPAN):
    for j in range(NCHORD):
        a = verts[idx(i,j)]
        b = verts[idx(i+1,j)]
        c = verts[idx(i+1,j+1)]
        d = verts[idx(i,j+1)]
        tris.append((a,b,c))
        tris.append((a,c,d))

# Binary STL
def write_stl(path, tris):
    with open(path,"wb") as f:
        f.write(b"\0"*80)
        f.write(struct.pack("<I", len(tris)))
        for (a,b,c) in tris:
            # normal
            ux,uy,uz = b[0]-a[0], b[1]-a[1], b[2]-a[2]
            vx,vy,vz = c[0]-a[0], c[1]-a[1], c[2]-a[2]
            nx,ny,nz = uy*vz-uz*vy, uz*vx-ux*vz, ux*vy-uy*vx
            n = math.sqrt(nx*nx+ny*ny+nz*nz) or 1.0
            f.write(struct.pack("<3f", nx/n, ny/n, nz/n))
            for v in (a,b,c):
                f.write(struct.pack("<3f", *v))
            f.write(b"\0\0")

stl_path = os.path.join(OUT_DIR,"canopy.stl")
write_stl(stl_path, tris)
print(f"OK {stl_path}  tris={len(tris)}  span={SPAN}m chord={CHORD}m")
