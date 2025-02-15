# camera-map-localization

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)

**Where is the car, given what the camera sees and what the map says?**

A readable C++17 implementation of map-matching localization on
[KITTI Odometry](http://www.cvlibs.net/datasets/kitti/eval_odometry.php): project
an HD map into the camera, score pose hypotheses against detected landmarks, and
fuse the best one into an error-state Kalman filter. No proprietary automotive
SDKs, no framework — plain CMake and Eigen, building on a laptop.

It is written to be **read**. Every non-obvious decision carries the reason it
was made, and where the implementation falls short of its own documentation, it
says so.

## Status

Early. Nothing here localizes anything yet.

## License

MIT — see [LICENSE](LICENSE).
