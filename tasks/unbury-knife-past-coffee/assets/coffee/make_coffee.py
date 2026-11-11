"""Measure the official mug and build a convex coffee frustum; numpy and pxr only.

Download SOURCE_URL separately, then pass that saved USD. No simulator or network
access is needed by this script. The official mug itself is not redistributed.
"""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from pxr import Gf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt

SOURCE_URL = "https://omniverse-content-staging.s3-us-west-2.amazonaws.com/Assets/Isaac/6.1/Isaac/IsaacLab/Objects/Mug/mug.usd"
MUG_EXTENT = 0.120
CLEARANCE = 0.002
SURFACE_BELOW_RIM = 0.008
SIDES = 32


def geometry(path):
    stage = Usd.Stage.Open(str(path))
    cache = UsdGeom.XformCache()
    points, faces = [], []
    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.Mesh):
            continue
        mesh = UsdGeom.Mesh(prim)
        p = np.asarray(mesh.GetPointsAttr().Get(), dtype=float)
        p = (np.column_stack((p, np.ones(len(p)))) @ np.asarray(cache.GetLocalToWorldTransform(prim)))[:, :3]
        offset = sum(len(part) for part in points)
        points.append(p)
        indices = np.asarray(mesh.GetFaceVertexIndicesAttr().Get(), dtype=int)
        start = 0
        for count in mesh.GetFaceVertexCountsAttr().Get():
            for k in range(1, count - 1):
                faces.append((offset + indices[start], offset + indices[start + k], offset + indices[start + k + 1]))
            start += count
    p = np.concatenate(points)
    lo, hi = p.min(axis=0), p.max(axis=0)
    scale = MUG_EXTENT / max((hi - lo)[:2])
    return (p - (lo + hi) / 2) * scale, np.asarray(faces), scale


def section(triangles, z):
    """XY line segments where the mug's actual triangles cross a horizontal plane."""
    t = triangles[(triangles[:, :, 2].min(axis=1) < z) & (triangles[:, :, 2].max(axis=1) > z)]
    a, b = t, np.roll(t, -1, axis=1)
    dz = b[:, :, 2] - a[:, :, 2]
    fraction = (z - a[:, :, 2]) / np.where(abs(dz) > 1e-15, dz, 1.0)
    hit = (abs(dz) > 1e-15) & (fraction >= 0) & (fraction < 1)
    valid = hit.sum(axis=1) == 2
    xy = a[:, :, :2] + fraction[:, :, None] * (b[:, :, :2] - a[:, :, :2])
    return xy[valid][hit[valid]].reshape(-1, 2, 2)


def radii(segments, axis, angles):
    p = segments[:, 0] - axis
    e = segments[:, 1] - segments[:, 0]
    direction = np.column_stack((np.cos(angles), np.sin(angles)))
    cross = lambda a, b: a[..., 0] * b[..., 1] - a[..., 1] * b[..., 0]
    det = cross(direction[:, None], e[None])
    safe = np.where(abs(det) > 1e-14, det, 1.0)
    r = cross(p, e)[None] / safe
    u = cross(p[None], direction[:, None]) / safe
    valid = (abs(det) > 1e-14) & (r > 1e-8) & (u >= 0) & (u <= 1)
    result = np.where(valid, r, np.inf).min(axis=1)
    if not np.isfinite(result).all():
        raise ValueError("A radial ray missed the mug; check the axis and source model")
    return result


def floor_at(triangles, xy, ceiling):
    """Highest mesh intersection below ceiling, measured vertically at xy."""
    t = triangles[triangles[:, :, 2].max(axis=1) < ceiling]
    a = t[:, 1, :2] - t[:, 0, :2]
    b = t[:, 2, :2] - t[:, 0, :2]
    q = np.asarray(xy) - t[:, 0, :2]
    det = a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0]
    safe = np.where(abs(det) > 1e-15, det, 1.0)
    u = (q[:, 0] * b[:, 1] - q[:, 1] * b[:, 0]) / safe
    v = (a[:, 0] * q[:, 1] - a[:, 1] * q[:, 0]) / safe
    hit = (abs(det) > 1e-15) & (u >= 0) & (v >= 0) & (u + v <= 1)
    z = t[:, 0, 2] + u * (t[:, 1, 2] - t[:, 0, 2]) + v * (t[:, 2, 2] - t[:, 0, 2])
    return float(z[hit].max())


def write_usd(path, bottom_radius, top_radius, height):
    stage = Usd.Stage.CreateNew(str(path))
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    root = UsdGeom.Xform.Define(stage, "/Coffee").GetPrim()
    stage.SetDefaultPrim(root)
    UsdPhysics.RigidBodyAPI.Apply(root).CreateRigidBodyEnabledAttr(True)
    UsdPhysics.MassAPI.Apply(root).CreateMassAttr(0.25)
    points = [(r * np.cos(a), r * np.sin(a), z) for r, z in
              ((bottom_radius, -height / 2), (top_radius, height / 2))
              for a in np.linspace(0, 2 * np.pi, SIDES, endpoint=False)]
    faces = [list(reversed(range(SIDES))), list(range(SIDES, 2 * SIDES))]
    faces += [[i, (i + 1) % SIDES, (i + 1) % SIDES + SIDES, i + SIDES] for i in range(SIDES)]
    mesh = UsdGeom.Mesh.Define(stage, "/Coffee/visual")
    mesh.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*p) for p in points]))
    mesh.CreateFaceVertexCountsAttr([len(f) for f in faces])
    mesh.CreateFaceVertexIndicesAttr([i for f in faces for i in f])
    mesh.CreateSubdivisionSchemeAttr("none")
    mesh.CreateExtentAttr(UsdGeom.PointBased(mesh).ComputeExtent(mesh.GetPointsAttr().Get()))
    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr("convexHull")
    material = UsdShade.Material.Define(stage, "/Coffee/Looks/Coffee")
    shader = UsdShade.Shader.Define(stage, "/Coffee/Looks/Coffee/Surface")
    shader.CreateIdAttr("UsdPreviewSurface")
    # USD diffuseColor is linear, task's (0.20, 0.11, 0.05) is sRGB.
    linear = [c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4 for c in (0.20, 0.11, 0.05)]
    from pxr import Sdf
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*linear))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.25)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material)
    stage.GetRootLayer().Save()


def build(source, output):
    output.mkdir(parents=True, exist_ok=True)
    points, faces, scale = geometry(source)
    triangles = points[faces]
    bottom = float(points[:, 2].min())
    rim_height = float(np.ptp(points[:, 2]))
    mid = section(triangles, bottom + rim_height * 0.6)
    axis = np.array([-0.017, 0.0])
    for _ in range(4):
        r = radii(mid, axis, np.arange(4) * np.pi / 2)
        axis += np.array([r[0] - r[2], r[1] - r[3]]) / 2
    floor = floor_at(triangles, axis, bottom + rim_height * 0.2) - bottom
    # Start 2 mm above the measured floor; the loose solid settles under gravity.
    base = floor + 0.002
    surface = rim_height - SURFACE_BELOW_RIM
    levels = np.linspace(base, surface, 129)
    angles = np.linspace(0, 2 * np.pi, 256, endpoint=False)
    profile = np.array([radii(section(triangles, bottom + z), axis, angles).min() for z in levels])
    ends = profile[[0, -1]] - CLEARANCE
    interpolated = np.linspace(*ends, len(levels))
    ends -= max(0.0, float((interpolated - (profile - CLEARANCE)).max())) + 1e-6
    # Independently check the floor under the entire base disk on a dense grid.
    floors = [floor_at(triangles, axis + [x, y], bottom + rim_height * 0.2) - bottom
              for x in np.linspace(-ends[0], ends[0], 13)
              for y in np.linspace(-ends[0], ends[0], 13) if x*x + y*y <= ends[0]**2]
    if max(floors) >= base - 0.0005:
        raise ValueError("Base intersects the mug floor; increase measured base clearance and rebuild")
    height = surface - base
    write_usd(output / "coffee.usda", *ends, height)
    info = {
        "source_url": SOURCE_URL,
        "source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
        "mug_extent_m": MUG_EXTENT, "mug_uniform_scale": scale,
        "mug_size_m": np.ptp(points, axis=0).tolist(),
        "mug_axis_from_bbox_center_m": [*axis.tolist(), 0.0],
        "mug_body_radius_m": float(np.ptp(points[:, 1]) / 2),
        "mug_rim_height_m": rim_height, "mug_inner_floor_m": floor,
        "coffee_bottom_above_mug_bottom_m": base,
        "coffee_surface_below_rim_m": SURFACE_BELOW_RIM,
        "coffee_height_m": height, "coffee_radii_m": ends.tolist(),
        "coffee_extent_m": float(2 * max(ends)),
        "wall_radial_clearance_m": float((profile - np.linspace(*ends, len(levels))).min()),
        "floor_gap_min_m": float(base - max(floors)),
        "profile_heights_above_mug_bottom_m": levels.tolist(),
        "profile_min_inner_radii_m": profile.tolist(),
        "measurement": "129 heights x 256 radial rays; floor checked on 13x13 disk grid; original quads triangulated by fan",
        "collision": "One 32-sided convex frustum (64 vertices), dynamic rigid body; no liquid simulation",
    }
    (output / "coffee_info.json").write_text(json.dumps(info, indent=2) + "\n")
    print(json.dumps({k: v for k, v in info.items() if not k.startswith("profile_")}, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mug_usd", type=Path)
    parser.add_argument("--output", type=Path, default=Path(__file__).resolve().parent)
    args = parser.parse_args()
    build(args.mug_usd.resolve(), args.output.resolve())
