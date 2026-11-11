# Fitted rigid coffee solid

`coffee.usda` is a generated 32-sided convex frustum, 250 g, with a dark-brown USD material. It is loose and
dynamic: neither fixed to the cup nor kinematic. It approximates coffee for pose-based spill scoring and does
not simulate fluid flow. Its 64 vertices fit within PhysX's convex-hull vertex limit.

The measurements come from Isaac Lab's official mug, at 120 mm width including the handle:

https://omniverse-content-staging.s3-us-west-2.amazonaws.com/Assets/Isaac/6.1/Isaac/IsaacLab/Objects/Mug/mug.usd

Source SHA-256: `7e71c247626c8bd17aa27dd7ad8aac1205cae7e9fc8a40fc911e0caf4f76ec80`.
The official mug is loaded separately by Arena; that source model is not redistributed here.

| Quantity | Measured/generated value |
|---|---|
| Mug width including handle / transverse width / height | 120 / 94.909 / 114.545 mm |
| Cup axis from mug bounding-box centre | (-17.082, 0.0004, 0) mm |
| Inner floor above mug mesh bottom, at cup axis | 6.894 mm |
| Coffee bottom / top diameter | 54.156 / 72.986 mm |
| Coffee height | 97.652 mm |
| Initial bottom above mug mesh bottom | 8.894 mm |
| Initial surface below rim | 8 mm |
| Minimum sampled radial wall clearance | 2.001 mm |
| Minimum sampled initial floor clearance | 1.990 mm |

`make_coffee.py` reads the original mesh with all ancestor transforms, triangulates its quad faces by fan,
normalizes the model to Arena's scaling, and measures horizontal cavity sections at 129 heights using 256
radial rays at each height. The frustum fits inside the smallest inner radius at each sampled height with
at least 2 mm clearance. The floor is also checked on a 13 by 13 grid clipped to the base disk. Full profiles
and source hash are saved in `coffee_info.json`. These are mesh checks; PhysX's cooked mug decomposition
and settled contacts must still be checked on GPU. The solid starts above the inner floor and settles under
gravity, so the final surface can be slightly lower than its starting height.

Arena's `Asset` type has no starting Z offset. The task's narrowly scoped `_install_coffee_placement()` adapter
adds the measured coffee-bottom elevation only for `coffee_fitted_local`, on top of Arena's normal 1 mm
rest gap. The cup uses the same rest gap. The USD itself contains only the actual solid; there are no hidden
support meshes, enlarged bounds or fixed joints. Installed Arena files are untouched.

To rebuild, download the source URL separately and run in an environment with `numpy` and `pxr`:

```sh
python tasks/unbury-knife-past-coffee/assets/coffee/make_coffee.py /path/to/mug.usd
```

Generated coffee geometry and script: MIT, like this task's code. Source measurements and attribution are
retained here and in the info file.
