# YCB bowl and plate

The models in `bowl/` and `plate/` come from the YCB Object and Model Set, Google 16k scans. They retain the
source meshes' UV coordinates, normals and diffuse texture images, and are licensed **CC BY 4.0**:

- Dataset: https://www.ycbbenchmarks.com/
- Licence: https://creativecommons.org/licenses/by/4.0/
- B. Calli, A. Walsman, A. Singh, S. Srinivasa, P. Abbeel and A. M. Dollar, "Benchmarking in Manipulation Research:
  Using the Yale-CMU-Berkeley Object and Model Set," *IEEE Robotics & Automation Magazine*, 22(3):36–52, 2015.

| Asset | Source | Maximum horizontal width | Height | Collision triangles |
|---|---|---|---|---|
| `bowl/bowl.usda` | `024_bowl/google_16k` | 160 mm | 54.51 mm | 16,378 |
| `plate/plate.usda` | `029_plate/google_16k` | 200 mm | 20.47 mm | 16,384 |

Each source consists of `textured.obj`, `textured.mtl` and `texture_map.png`. The supplied archive is a gzip
compressed tar archive; the `.gz` filename is accepted directly. The texture bytes are copied unchanged to
`bowl_texture.png` and `plate_texture.png`. Each model's `*_info.json` records its source hashes, uniform scale,
dimensions, centre surface height and collision configuration.

## Conversion and physics

`make_ycb_containers.py` uniformly scales the scans, keeps their existing Z-up orientation with openings toward
+Z, and places each bounding-box centre at the local origin. USD units are metres. Arena places the models on
the table using their bounds and the widths already declared in `task.py`.

Both roots have `PhysicsRigidBodyAPI` with `physics:kinematicEnabled = true`, matching their fixed, massless
`Asset` specifications in `task.py`. Their separate guide meshes use `PhysicsCollisionAPI` and
`PhysicsMeshCollisionAPI` with `physics:approximation = "none"`: the collision surface follows the original
triangles, including the bowl interior and the plate's recessed surface. It does not fill the opening with a
convex hull. Six zero-area facets in the bowl scan are omitted from collision; the visual mesh is retained.

The highest centre surface is 2.37 mm above the bowl's mesh bottom and 2.75 mm above the plate's mesh bottom.
The 20.47 mm plate height is its overall rim height, not the height of its centre surface. The existing demo's
placement height and the actual contact heights still need GPU calibration.

## Rebuild

From the repository root, in an environment with `numpy` and `usd-core` (`pxr`):

```sh
python tasks/unbury-knife-past-coffee/assets/make_ycb_containers.py /path/to/ycb_bowl_plate.gz
```

Use `--output /path/to/assets` to rebuild elsewhere. The script reads only the six named archive members and
does not extract arbitrary archive paths. Textures use relative USD asset paths so the task folder can be moved
to the GPU machine as a whole. CPU tests inspect the saved USD dimensions, texture resolution and the cavity's
collision depth; simulator contact behaviour remains to be verified on GPU.
