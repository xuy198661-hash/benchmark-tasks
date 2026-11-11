"""Build knife.usda from the YCB 032_knife scan (google_16k/textured.obj).

Usage: python make_knife_usda.py path/to/032_knife/google_16k/textured.obj
Writes knife.usda and knife_info.json next to this script; copy texture_map.png here as knife_texture.png.
Needs numpy and scipy.
"""
import json
import pathlib
import sys

import numpy as np
from scipy.spatial import ConvexHull

OUT = pathlib.Path(__file__).resolve().parent
SRC = sys.argv[1]
LENGTH = 0.170
HANDLE_FROM_X = -0.020  # original-scan x where the handle starts (thick part runs from here to +88 mm)

V, VT, VN, F = [], [], [], []
for line in open(SRC):
    p = line.split()
    if not p:
        continue
    if p[0] == "v":
        V.append([float(t) for t in p[1:4]])
    elif p[0] == "vt":
        VT.append([float(t) for t in p[1:3]])
    elif p[0] == "vn":
        VN.append([float(t) for t in p[1:4]])
    elif p[0] == "f":
        F.append([[int(i) - 1 for i in c.split("/")] for c in p[1:]])
V, VT, VN = np.array(V), np.array(VT), np.array(VN)
assert all(len(f) == 3 for f in F)
F = np.array(F)  # (nf, 3 corners, [v, vt, vn])
handle_mask_orig = V[:, 0] >= HANDLE_FROM_X
tip_end_orig = V[np.argmin(V[:, 0])]

# Volume centroid (solid, from the closed triangle mesh) as the centre of mass.
tri = V[F[:, :, 0]]
vol = np.einsum("ij,ij->i", tri[:, 0], np.cross(tri[:, 1], tri[:, 2])) / 6.0
com = (vol[:, None] * tri.sum(1) / 4.0).sum(0) / vol.sum()

# Stable resting face: the hull facet whose plane is closest to the COM and contains its projection.
hull = ConvexHull(V)
best = None
for simplex, eq in zip(hull.simplices, hull.equations):
    n, d = eq[:3], eq[3]
    if n[2] > -0.9:  # keep the scan's underside down: the lowest face overall is the knife flipped upside down
        continue
    dist = -(n @ com + d)
    proj = com - (n @ com + d) * n
    a, b, c = V[simplex]
    # barycentric test on the facet's support plane (use the whole coplanar region approximately via the facet)
    v0, v1, v2 = b - a, c - a, proj - a
    d00, d01, d11, d20, d21 = v0 @ v0, v0 @ v1, v1 @ v1, v2 @ v0, v2 @ v1
    den = d00 * d11 - d01 * d01
    if den <= 0:
        continue
    s = (d11 * d20 - d01 * d21) / den
    t = (d00 * d21 - d01 * d20) / den
    if s >= -1e-9 and t >= -1e-9 and s + t <= 1 + 1e-9 and (best is None or dist < best[0]):
        best = (dist, n)
assert best is not None, "no stable facet"
n_down = best[1]  # outward normal of the face it rests on -> should point to -z


def rot_between(a, b):
    a, b = a / np.linalg.norm(a), b / np.linalg.norm(b)
    v, c = np.cross(a, b), a @ b
    if np.linalg.norm(v) < 1e-12:
        return np.eye(3) if c > 0 else -np.eye(3)
    k = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + k + k @ k * (1 / (1 + c))


R1 = rot_between(n_down, np.array([0.0, 0.0, -1.0]))
P = V @ R1.T
# Yaw so the long axis (handle end -> tip) points to +y.
tip1 = tip_end_orig @ R1.T
c2 = P.mean(0)
xy = P[:, :2] - c2[:2]
w, vec = np.linalg.eigh(np.cov(xy.T))
axis = vec[:, -1]
if axis @ (tip1[:2] - c2[:2]) < 0:
    axis = -axis
ang = np.arctan2(axis[0], axis[1])  # rotate axis onto +y
cz, sz = np.cos(ang), np.sin(ang)
R2 = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
R = R2 @ R1
P = V @ R.T
assert P[np.argmin(V[:, 0])][1] > 0, "tip must be at +y"
scale = LENGTH / float(np.max(P.max(0)[:2] - P.min(0)[:2]))
P = P * scale
lo, hi = P.min(0), P.max(0)
centre = (lo + hi) / 2
P = P - centre
lo, hi = P.min(0), P.max(0)
N = VN @ R.T
N /= np.linalg.norm(N, axis=1, keepdims=True)

# Read off the textured render (knife_check.png): the red handle ends at y = -17 mm, a thin metal neck runs to
# y = +3 mm, and the blade from there to the tip.
HANDLE_END_Y = -0.017
BLADE_START_Y = 0.003
handle = P[P[:, 1] <= HANDLE_END_Y]
blade = P[P[:, 1] >= BLADE_START_Y]
boundary_y = HANDLE_END_Y


def hull_mesh(pts):
    h = ConvexHull(pts)
    idx = sorted(set(h.vertices))
    remap = {o: i for i, o in enumerate(idx)}
    faces = []
    cen = pts[idx].mean(0)
    for s in h.simplices:
        a, b, c = pts[s]
        if np.cross(b - a, c - a) @ (a - cen) < 0:
            s = [s[0], s[2], s[1]]
        faces.append([remap[i] for i in s])
    return pts[idx], faces


# A little overlap so the two hulls meet with no gap at the boundary.
hh = hull_mesh(P[P[:, 1] <= HANDLE_END_Y + 0.001])
nh = hull_mesh(P[(P[:, 1] >= HANDLE_END_Y - 0.001) & (P[:, 1] <= BLADE_START_Y + 0.001)])
bh = hull_mesh(P[P[:, 1] >= BLADE_START_Y - 0.001])

info = {
    "scale_from_scan": scale,
    "size": (hi - lo).tolist(),
    "bbox_low": lo.tolist(),
    "bbox_high": hi.tolist(),
    "tip_y": float(hi[1]),
    "butt_y": float(lo[1]),
    "handle_end_y": HANDLE_END_Y,
    "blade_start_y": BLADE_START_Y,
    "handle_len": float(HANDLE_END_Y - lo[1]),
    "blade_len": float(hi[1] - BLADE_START_Y),
    "handle_width_mm": float(np.ptp(handle[:, 0]) * 1000),
    "handle_height_mm": float(np.ptp(handle[:, 2]) * 1000),
    "blade_height_mm": float(np.ptp(blade[:, 2]) * 1000),
    "com": ((com @ R.T) * scale - centre).tolist(),
    "rest_tilt_deg": float(np.degrees(np.arccos(abs(n_down @ np.array([0, 0, -1.0]))))),
    "hull_points": [len(hh[0]), len(nh[0]), len(bh[0])],
    "neck_height_mm": float(np.ptp(nh[0][:, 2]) * 1000),
    "neck_width_mm": float(np.ptp(nh[0][:, 0]) * 1000),
}
print(json.dumps(info, indent=1))
json.dump(info, open(OUT / "knife_info.json", "w"), indent=1)
g = {"hh_p": hh[0], "hh_f": np.array(hh[1]), "nh_p": nh[0], "nh_f": np.array(nh[1]),
     "bh_p": bh[0], "bh_f": np.array(bh[1])}

# -- write the USD ---------------------------------------------------------------------------------------------------
MASS = 0.050


def arr(rows, fmt):
    return ", ".join(fmt % tuple(r) for r in rows)


def ints(a):
    return ", ".join(str(int(i)) for i in np.ravel(a))


def hull(name, pts, faces):
    return f'''
    def Mesh "{name}" (
        prepend apiSchemas = ["PhysicsCollisionAPI", "PhysicsMeshCollisionAPI"]
    )
    {{
        uniform token physics:approximation = "convexHull"
        uniform token purpose = "guide"
        int[] faceVertexCounts = [{ints([3] * len(faces))}]
        int[] faceVertexIndices = [{ints(faces)}]
        point3f[] points = [{arr(pts, "(%.6f, %.6f, %.6f)")}]
        uniform token subdivisionScheme = "none"
    }}
'''


HULLS = (hull("collision_handle", g["hh_p"], g["hh_f"]) + hull("collision_neck", g["nh_p"], g["nh_f"])
         + hull("collision_blade", g["bh_p"], g["bh_f"]))
text = f'''#usda 1.0
(
    defaultPrim = "Knife"
    metersPerUnit = 1
    upAxis = "Z"
    doc = """Knife from the YCB Object and Model Set (032_knife, google_16k scan), CC BY 4.0.
Resting on its underside, tip toward +y, scaled to 170 mm, bounding-box centre at the origin.
Red handle: y < -17 mm; metal neck: -17 to +3 mm; blade: y > +3 mm. Collision: one convex hull each for the
handle, the neck and the blade. The scan adds a few millimetres of spurious thickness under the polished blade."""
)

def Xform "Knife" (
    prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI"]
)
{{
    float physics:mass = {MASS}

    def Mesh "visual" (
        prepend apiSchemas = ["MaterialBindingAPI"]
    )
    {{
        int[] faceVertexCounts = [{ints([3] * len(F))}]
        int[] faceVertexIndices = [{ints(F[:, :, 0])}]
        point3f[] points = [{arr(P, "(%.6f, %.6f, %.6f)")}]
        texCoord2f[] primvars:st = [{arr(VT, "(%.6f, %.6f)")}] (
            interpolation = "faceVarying"
        )
        int[] primvars:st:indices = [{ints(F[:, :, 1])}]
        normal3f[] primvars:normals = [{arr(N, "(%.5f, %.5f, %.5f)")}] (
            interpolation = "faceVarying"
        )
        int[] primvars:normals:indices = [{ints(F[:, :, 2])}]
        uniform token subdivisionScheme = "none"
        rel material:binding = </Knife/Looks/Scan>
    }}
{HULLS}
    def Scope "Looks"
    {{
        def Material "Scan"
        {{
            token outputs:surface.connect = </Knife/Looks/Scan/Surface.outputs:surface>

            def Shader "Surface"
            {{
                uniform token info:id = "UsdPreviewSurface"
                color3f inputs:diffuseColor.connect = </Knife/Looks/Scan/Texture.outputs:rgb>
                float inputs:metallic = 0.3
                float inputs:roughness = 0.35
                token outputs:surface
            }}

            def Shader "Texture"
            {{
                uniform token info:id = "UsdUVTexture"
                asset inputs:file = @./knife_texture.png@
                float2 inputs:st.connect = </Knife/Looks/Scan/StReader.outputs:result>
                token inputs:sourceColorSpace = "sRGB"
                float3 outputs:rgb
            }}

            def Shader "StReader"
            {{
                uniform token info:id = "UsdPrimvarReader_float2"
                string inputs:varname = "st"
                float2 outputs:result
            }}
        }}
    }}
}}
'''
open(OUT / "knife.usda", "w").write(text)
print("written", len(text) // 1024, "KiB")
