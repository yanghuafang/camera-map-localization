# camera-map-localization
C++17 reference implementation of camera map-matching localization: rasterize lane/road-boundary perception to distance transforms, score a 3-DOF (x, y, yaw) pose grid against a polyline map, temporally aggregate, and fuse into an SE(3) Kalman filter. Optional CUDA; KITTI eval, benchmarks, and RViz/PNG visualization.
