# Calibrated Camera Projection

Independent Tech Boss geometry demo: a transparent pinhole-camera implementation with world-to-camera transforms and clipping. This is not a learned reconstruction system, calibrated physical device or client project.

```sh
python project.py --output examples
python -m unittest discover -s tests -v
```

Run from this directory with Python 3.10+. Standard library only; no GPU, downloads or API keys.

The camera frame uses x right, y down, z forward. Intrinsics map camera-space points to pixels with `u = fx*x/z + cx` and `v = fy*y/z + cy`. The world-to-camera transform first subtracts camera position, then applies inverse yaw. Distances use consistent arbitrary scene units; focal lengths and principal point use pixels.

Edges are clipped in 3D against the near, left, right, top and bottom half-spaces before perspective division. This prevents coordinates from exploding when a line crosses z=0. Frustum image bounds are continuous edges `[0,width] × [0,height]`, not discrete pixel indices. The scene deliberately contains an off-screen cube and a cube behind the camera.

Unclipped endpoints are preserved exactly, so floating-point interpolation cannot inflate the partial-clipping count. Near-plane intersections are snapped only within a relative tolerance of `1e-12`; points meaningfully behind the plane remain rejected. Dedicated regression tests cover fractional near-plane crossings and exact endpoint preservation.

`examples/projection.svg` is an actual CLI-generated browser-viewable rendering. `examples/report.json` records camera parameters and rejected/visible edge counts. These counts are geometry diagnostics, **not model accuracy**. Unit tests validate analytic projections, scale invariance, inverse pose, near-plane clipping, complete rejection, frame crossing and edge reversal.

Limits: yaw-only pose, no lens distortion, no camera calibration estimation, no texture/rasterization, no z-buffer/occlusion, and no image-based inference. Overlapping edges remain visible in this wireframe renderer. See [MODEL_CARD.md](MODEL_CARD.md).
