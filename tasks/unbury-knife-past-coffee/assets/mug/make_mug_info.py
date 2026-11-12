"""Measure Isaac Lab's official mug for the task's scoring geometry; numpy and pxr only.

Download SOURCE_URL separately, then pass that saved USD. No simulator or network access is needed by this script.
The official mug itself is not redistributed. Writes mug_info.json next to this script.
"""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from pxr import Usd, UsdGeom

SOURCE_URL = "https://omniverse-content-staging.s3-us-west-2.amazonaws.com/Assets/Isaac/6.1/Isaac/IsaacLab/Objects/Mug/mug.usd"
MUG_EXTENT = 0.120  # m, overall width including the handle, as declared in task.py


def geometry(path):
    """All mesh triangles in world space, scaled the way Arena scales the asset (longest horizontal side = extent)."""
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
    """XY line segments where the mug's triangles cross a horizontal plane."""
    t = triangles[(triangles[:, :, 2].min(axis=1) < z) & (triangles[:, :, 2].max(axis=1) > z)]
    a, b = t, np.roll(t, -1, axis=1)
    dz = b[:, :, 2] - a[:, :, 2]
    fraction = (z - a[:, :, 2]) / np.where(abs(dz) > 1e-15, dz, 1.0)
    hit = (abs(dz) > 1e-15) & (fraction >= 0) & (fraction < 1)
    valid = hit.sum(axis=1) == 2
    xy = a[:, :, :2] + fraction[:, :, None] * (b[:, :, :2] - a[:, :, :2])
    return xy[valid][hit[valid]].reshape(-1, 2, 2)


def radii(segments, axis, angles):
    """Distance from the axis to the nearest wall along each horizontal ray."""
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


def measure(source):
    points, faces, scale = geometry(source)
    triangles = points[faces]
    bottom = float(points[:, 2].min())
    rim_height = float(np.ptp(points[:, 2]))
    # Centre the cup axis between opposite inner walls at 60 % of the height (the handle is toward +x).
    mid = section(triangles, bottom + rim_height * 0.6)
    axis = np.array([-0.017, 0.0])
    for _ in range(4):
        r = radii(mid, axis, np.arange(4) * np.pi / 2)
        axis += np.array([r[0] - r[2], r[1] - r[3]]) / 2
    floor = floor_at(triangles, axis, bottom + rim_height * 0.2) - bottom
    return {
        "source_url": SOURCE_URL,
        "source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
        "mug_extent_m": MUG_EXTENT,
        "mug_uniform_scale": scale,
        "mug_size_m": np.ptp(points, axis=0).tolist(),
        "mug_axis_from_bbox_center_m": [*axis.tolist(), 0.0],
        "mug_body_radius_m": float(np.ptp(points[:, 1]) / 2),
        "mug_rim_height_m": rim_height,
        "mug_inner_floor_m": floor,
        "measurement": "mesh only: quads triangulated by fan; axis centred between opposite inner walls at 60 % height",
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mug_usd", type=Path)
    parser.add_argument("--output", type=Path, default=Path(__file__).resolve().parent)
    args = parser.parse_args()
    info = measure(args.mug_usd.resolve())
    (args.output / "mug_info.json").write_text(json.dumps(info, indent=2) + "\n")
    print(json.dumps(info, indent=2))
