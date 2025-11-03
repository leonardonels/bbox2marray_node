# localisation

Purpose
-------
Convert 2D bounding boxes produced by the detector into 3D points using the
ZED depth map and publish them as marker messages for downstream mapping.

Key files
- `src/bbox2marker_node.cpp` — node that subscribes to detections and depth.
- `src/bbox2marker.cpp`, `include/bbox2marker/bbox2marker.hpp` — core logic
  for projection and depth sampling.
- `launch/launch.py` — launch helpers and parameter loading.

Topics
- Subscribes: bounding boxes from `yolo/`, depth or point cloud from the
  ZED driver.
- Publishes: `visualization_msgs/Marker` or `MarkerArray` containing 3D points
  for each detected cone.

Run (example)
--------------
```bash
ros2 launch localisation launch.py
```

Important notes
- Ensure the camera intrinsics and TF frames are correct and nodes are
  synchronized (image <-> depth). If depth is missing, localisation cannot
  compute valid 3D points.
- The node may sample several pixels inside a bbox and filter invalid depth
  values (NaN, inf, out of range).