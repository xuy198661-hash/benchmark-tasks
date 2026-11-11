# Knife model

`knife.usda` and `knife_texture.png` come from the YCB Object and Model Set, object `032_knife`, Google 16k
scan (`google_16k/textured.obj` and `texture_map.png`), licensed CC BY 4.0:

B. Calli, A. Walsman, A. Singh, S. Srinivasa, P. Abbeel and A. M. Dollar, "Benchmarking in Manipulation Research:
Using the Yale-CMU-Berkeley Object and Model Set," *IEEE Robotics & Automation Magazine*, 22(3):36-52, 2015.

Changes made by `make_knife_usda.py`:

- Turned so the knife rests on the face it lay on when scanned (the hull face nearest the centre of mass with the
  centre of mass above it), with the tip toward +y.
- Scaled to 170 mm long (the scan is 215 mm), with the bounding-box centre at the origin.
- Rigid body of 50 g, colliding as three convex hulls: the red handle (y < -17 mm), the metal neck (-17 to +3 mm)
  and the blade (y > +3 mm). The hulls are guide geometry, not rendered.

The scan adds a few millimetres of spurious thickness under the polished blade; the blade's hull keeps it.

Rebuild:

```
python make_knife_usda.py path/to/032_knife/google_16k/textured.obj
cp path/to/032_knife/google_16k/texture_map.png knife_texture.png
```
