"""Convert the YCB bowl/plate archive to textured, concave USD fixtures.

Usage: python make_ycb_containers.py /path/to/ycb_bowl_plate.gz
Requires numpy and usd-core (pxr). No Isaac Sim or external mesh cooker is needed.
"""

import argparse
import hashlib
import json
from pathlib import Path
import tarfile

import numpy as np
from pxr import Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt


MODELS = (("bowl", "024_bowl", 0.160), ("plate", "029_plate", 0.200))
CITATION = (
    'B. Calli et al., "Benchmarking in Manipulation Research: Using the Yale-CMU-Berkeley '
    'Object and Model Set," IEEE Robotics & Automation Magazine 22(3):36-52, 2015.'
)


def read_member(archive, name):
    member = archive.getmember(name)
    if not member.isfile():
        raise ValueError(f"Expected a regular archive member: {name}")
    return archive.extractfile(member).read()


def read_obj(data):
    vertices, uvs, normals, faces = [], [], [], []
    for line in data.decode("utf-8").splitlines():
        parts = line.split()
        if not parts:
            continue
        if parts[0] == "v":
            vertices.append([float(x) for x in parts[1:4]])
        elif parts[0] == "vt":
            uvs.append([float(x) for x in parts[1:3]])
        elif parts[0] == "vn":
            normals.append([float(x) for x in parts[1:4]])
        elif parts[0] == "f":
            if len(parts) != 4 or any(len(p.split("/")) != 3 for p in parts[1:]):
                raise ValueError("Expected triangulated faces with vertex/UV/normal indices")
            faces.append([[int(x) - 1 for x in p.split("/")] for p in parts[1:]])
    vertices, uvs, normals = (np.asarray(a, dtype=float) for a in (vertices, uvs, normals))
    faces = np.asarray(faces, dtype=np.int32)
    for i, (label, values, width) in enumerate(
        (("vertices", vertices, 3), ("UVs", uvs, 2), ("normals", normals, 3))
    ):
        if values.ndim != 2 or values.shape[1] != width or not np.isfinite(values).all():
            raise ValueError(f"Invalid OBJ {label}")
        if faces.ndim != 3 or faces.shape[1:] != (3, 3):
            raise ValueError("Invalid OBJ faces")
        if faces[:, :, i].min() < 0 or faces[:, :, i].max() >= len(values):
            raise ValueError(f"Invalid OBJ indices for {label}")
    return vertices, uvs, normals, faces


def vec3s(values):
    return Vt.Vec3fArray.FromNumpy(np.asarray(values, dtype=np.float32))


def define_mesh(stage, path, points, faces):
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr(vec3s(points))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray([3] * len(faces)))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(faces.astype(np.int32).ravel()))
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    mesh.CreateExtentAttr(vec3s(np.asarray([points.min(0), points.max(0)])))
    return mesh


def centre_surface_height(points, faces):
    """Topmost vertical-ray intersection at the centre, measured from the mesh bottom."""
    triangles = points[faces]
    u = triangles[:, 1, :2] - triangles[:, 0, :2]
    v = triangles[:, 2, :2] - triangles[:, 0, :2]
    q = -triangles[:, 0, :2]
    det = u[:, 0] * v[:, 1] - u[:, 1] * v[:, 0]
    valid = abs(det) > 1e-14
    denominator = np.where(valid, det, 1.0)
    b = (q[:, 0] * v[:, 1] - q[:, 1] * v[:, 0]) / denominator
    c = (u[:, 0] * q[:, 1] - u[:, 1] * q[:, 0]) / denominator
    hit = valid & (b >= -1e-10) & (c >= -1e-10) & (b + c <= 1 + 1e-10)
    if not hit.any():
        raise ValueError("No surface under the container centre")
    z = triangles[:, 0, 2] + b * (triangles[:, 1, 2] - triangles[:, 0, 2])
    z += c * (triangles[:, 2, 2] - triangles[:, 0, 2])
    return float(z[hit].max() - points[:, 2].min())


def build(archive, output, name, source, width):
    prefix = f"{source}/google_16k"
    obj = read_member(archive, f"{prefix}/textured.obj")
    mtl = read_member(archive, f"{prefix}/textured.mtl")
    texture = read_member(archive, f"{prefix}/texture_map.png")
    if "map_Kd texture_map.png" not in mtl.decode("utf-8"):
        raise ValueError(f"Unexpected texture binding in {source}")
    vertices, uvs, normals, faces = read_obj(obj)
    low, high = vertices.min(0), vertices.max(0)
    scale = width / max((high - low)[:2])
    # The supplied YCB scans are already Z-up, with the openings toward +Z.
    points = (vertices - (low + high) / 2) * scale
    # Match the float32 coordinates actually authored in USD before checking facets.
    points = points.astype(np.float32).astype(float)
    triangles = points[faces[:, :, 0]]
    area2 = np.linalg.norm(
        np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0]), axis=1
    )
    collision_faces = faces[area2 > 1e-14, :, 0]
    surface = centre_surface_height(points, collision_faces)
    size = points.max(0) - points.min(0)
    if surface >= size[2] / 2:
        raise ValueError(f"{source}: expected an open container with a recessed centre")

    folder = output / name
    folder.mkdir(parents=True, exist_ok=True)
    usd_path = folder / f"{name}.usda"
    stage = Usd.Stage.CreateNew(str(usd_path))
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    root_path = f"/{name.title()}"
    root = UsdGeom.Xform.Define(stage, root_path).GetPrim()
    stage.SetDefaultPrim(root)
    root.SetDocumentation(
        f"YCB {source}, Google 16k scan; CC BY 4.0. {CITATION} "
        f"Uniformly scaled to {width * 1000:g} mm wide and centred on its bounding box. "
        "Kinematic fixture with the original concave triangle surface for collision."
    )
    rigid = UsdPhysics.RigidBodyAPI.Apply(root)
    rigid.CreateRigidBodyEnabledAttr(True)
    rigid.CreateKinematicEnabledAttr(True)

    visual = define_mesh(stage, f"{root_path}/visual", points, faces[:, :, 0])
    visual.CreateNormalsAttr(vec3s(normals[faces[:, :, 2]].reshape(-1, 3)))
    visual.SetNormalsInterpolation(UsdGeom.Tokens.faceVarying)
    st = UsdGeom.PrimvarsAPI(visual).CreatePrimvar(
        "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying
    )
    st.Set(Vt.Vec2fArray.FromNumpy(uvs.astype(np.float32)))
    st.SetIndices(Vt.IntArray.FromNumpy(faces[:, :, 1].ravel()))

    collider = define_mesh(stage, f"{root_path}/collision", points, collision_faces)
    collider.CreatePurposeAttr(UsdGeom.Tokens.guide)
    UsdPhysics.CollisionAPI.Apply(collider.GetPrim()).CreateCollisionEnabledAttr(True)
    UsdPhysics.MeshCollisionAPI.Apply(collider.GetPrim()).CreateApproximationAttr(UsdPhysics.Tokens.none)

    material = UsdShade.Material.Define(stage, f"{root_path}/Looks/Scan")
    surface_shader = UsdShade.Shader.Define(stage, f"{material.GetPath()}/Surface")
    surface_shader.CreateIdAttr("UsdPreviewSurface")
    surface_shader.CreateOutput("surface", Sdf.ValueTypeNames.Token)
    material.CreateSurfaceOutput().ConnectToSource(surface_shader.ConnectableAPI(), "surface")
    texture_shader = UsdShade.Shader.Define(stage, f"{material.GetPath()}/Texture")
    texture_shader.CreateIdAttr("UsdUVTexture")
    texture_shader.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(f"./{name}_texture.png")
    texture_shader.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB")
    texture_shader.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
    surface_shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(
        texture_shader.ConnectableAPI(), "rgb"
    )
    reader = UsdShade.Shader.Define(stage, f"{material.GetPath()}/StReader")
    reader.CreateIdAttr("UsdPrimvarReader_float2")
    reader.CreateInput("varname", Sdf.ValueTypeNames.String).Set("st")
    reader.CreateOutput("result", Sdf.ValueTypeNames.Float2)
    texture_shader.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(reader.ConnectableAPI(), "result")
    UsdShade.MaterialBindingAPI.Apply(visual.GetPrim()).Bind(material)
    (folder / f"{name}_texture.png").write_bytes(texture)
    stage.GetRootLayer().Save()

    info = {
        "source": prefix,
        "source_website": "https://www.ycbbenchmarks.com/",
        "license": "CC BY 4.0",
        "license_url": "https://creativecommons.org/licenses/by/4.0/",
        "citation": CITATION,
        "source_sha256": {suffix: hashlib.sha256(data).hexdigest()
                          for suffix, data in (("textured.obj", obj), ("textured.mtl", mtl),
                                               ("texture_map.png", texture))},
        "scale_from_scan": float(scale),
        "size": size.tolist(),
        "bbox_low": points.min(0).tolist(),
        "bbox_high": points.max(0).tolist(),
        "centre_surface_height_from_bottom": surface,
        "cavity_depth_from_highest_rim": float(size[2] - surface),
        "visual_triangles": len(faces),
        "collision_triangles": len(collision_faces),
        "collision_degenerate_triangles_removed": len(faces) - len(collision_faces),
        "collision_approximation": "none",
        "kinematic": True,
    }
    (folder / f"{name}_info.json").write_text(json.dumps(info, indent=2) + "\n")
    print(f"{name}: size {size * 1000} mm; {len(collision_faces)} collision triangles; {usd_path}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("archive", type=Path)
    parser.add_argument("--output", type=Path, default=Path(__file__).resolve().parent)
    args = parser.parse_args()
    with tarfile.open(args.archive, "r:gz") as archive:
        for name, source, width in MODELS:
            build(archive, args.output, name, source, width)


if __name__ == "__main__":
    main()
