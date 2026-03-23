### Camera (Intrinsic) Matrix K
The camera matrix (camera_matrix) maps 3D camera-space points to 2D image pixels:

```
camera_matrix:
  rows: 3
  cols: 3
  data: [  fx      ,    0.     ,   cx,
            0.     ,    fy     ,   cy,
            0.     ,    0.     ,    1.     ]
```

K = [ fx   0   cx ]
    [  0  fy   cy ]
    [  0   0    1 ]

In your file:

Camera.fxl → focal length in pixels along X
Camera.fyl → focal length in pixels along Y
Camera.cxl → principal point X (optical center column)
Camera.cyl → principal point Y (optical center row)
Projection formula: [u, v, 1]ᵀ = K * [X/Z, Y/Z, 1]ᵀ

### Distortion Coefficients D
Real lenses distort light. OpenCV's plumb-bob model (used here) has:

```
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [k1, k2, p1, p2, k3]
```

D = [k1, k2, p1, p2 (, k3)]
k1, k2 (Camera.k1l, Camera.k2l) — radial distortion (barrel/pincushion). Applied as: r² = x² + y², correction ≈ (1 + k1·r² + k2·r⁴)

p1, p2 (Camera.p1l, Camera.p2l) — tangential distortion (lens not perfectly parallel to sensor)

These are applied before the camera matrix, to correct the raw distorted pixel coordinates back to ideal pinhole positions.

### Projection Matrix P (stereo, 3×4)
Used after stereo rectification. It combines K with the stereo baseline:


Left:   P = [ fx   0   cx    0  ]
            [  0  fy   cy    0  ]
            [  0   0    1    0  ]

Right:  P = [ fx   0   cx  -fx·Tx ]
            [  0  fy   cy    0    ]
            [  0   0    1    0    ]
            
Where Tx is the baseline (distance between cameras). Your file sets bdo_stereo_rect: 0, so OV2SLAM is using the raw camera matrices + distortion directly rather than rectified projection matrices.