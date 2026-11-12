# Mug measurements

The task uses Isaac Lab's official mug, loaded by Arena from its asset server; the model itself is not
redistributed here. Source:

https://omniverse-content-staging.s3-us-west-2.amazonaws.com/Assets/Isaac/6.1/Isaac/IsaacLab/Objects/Mug/mug.usd

Source SHA-256: `7e71c247626c8bd17aa27dd7ad8aac1205cae7e9fc8a40fc911e0caf4f76ec80`.

`mug_info.json` holds the measurements `task.py` reads, taken from the mesh at Arena's scale (120 mm overall width,
including the handle):

| Quantity | Value |
|---|---|
| Width including handle / transverse width / height | 120 / 94.909 / 114.545 mm |
| Cup axis from the bounding-box centre | (-17.082, 0.0004, 0) mm |
| Body radius (half the transverse width) | 47.455 mm |
| Inner floor above the mesh bottom, at the cup axis | 6.894 mm |

The coffee is not modelled; the task gives the mug the mass of a full cup and judges spilling from its tilt and
shift. These are mesh measurements; the collision shape PhysX builds for the dynamic mug is not checked here.

To rebuild, download the source URL separately and run in an environment with `numpy` and `usd-core` (`pxr`):

```sh
python tasks/unbury-knife-past-coffee/assets/mug/make_mug_info.py /path/to/mug.usd
```

Script: MIT, like this task's code.
