# Official Wuji Hand 2.2 resources

Source: https://github.com/wuji-technology/wuji-description,
tag `v2026.10.12`, commit `829c9aa27cb12c3a49d5b8d63c9869dd8f656702`,
`hand2/hand2_2/body/urdf/{left,right}.urdf` and their referenced meshes.
Copyright (c) 2025 Wuji Technology, MIT; see `LICENSE.txt`.
`hand2_2/source.json` records original source hashes.

Each side contains its standalone URDF with XML serialization normalized and
mesh filenames rewritten from `../meshes/<side>/<name>.STL` to
`meshes/<name>.STL`. Only referenced meshes are copied, unchanged. Joint names,
types, origins, axes, limits, inertias, materials and fixed fingertip/IMU frames
are preserved. No combined benchmark robot was used as the source.

`end_effector.yaml` drives the 20 upstream joints in finger-major order:
thumb CMC flex/CMC abd/MCP/IP, then each finger's MCP flex/MCP abd/PIP/DIP.
Inputs are radians with `at_0: 0`, `at_1: 1`, `clip: false`; neither side uses
normalized gripper coordinates or an extra sign conversion. The fixed TCP
matches the original `<side>_wrist_fixed` transform, a CAD palm reference.
Installation mounting and an optional calibrated TCP belong to the assembly.

The upstream model includes provisional pad masses. Neither these inertias nor
the joint limits establish measured installation loads, a collision model or
hardware-qualified command limits.
