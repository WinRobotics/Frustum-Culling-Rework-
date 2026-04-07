# Frustum Culling Rework

A simple PCL-based tool that filters point clouds to keep only points within a camera's field of view, using `pcl::FrustumCulling`. Loads `.pcd` files from a directory, applies the frustum filter, overwrites them in place, and visualizes the result.

## Requirements

- Ubuntu 22.04
- CMake ≥ 3.10
- PCL 1.12 (ships with 22.04)
- Eigen3
- libx11-dev (for the PCL visualizer)

## Install dependencies
```bash
sudo apt update
sudo apt install -y \
    build-essential \
    cmake \
    libpcl-dev \
    libeigen3-dev \
    libx11-dev
```

## Build

From the workspace root:
```bash
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

## Run

Pass the path to a directory containing `.pcd` files as the only argument:
```bash
./frustum_culling ../pcds
```

The program will:

1. Load every `.pcd` file in the given directory.
2. Strip NaN points.
3. Apply frustum culling with the configured FOV, near/far planes, and camera pose.
4. Overwrite each `.pcd` with the filtered cloud.
5. Open a PCL visualizer showing the first filtered cloud. Close the window to exit.

## Tuning the frustum

The frustum parameters live in `filter()` inside the source file:
```cpp
fc.setVerticalFOV(100.0);
fc.setHorizontalFOV(100.0);
fc.setNearPlaneDistance(0.0);
fc.setFarPlaneDistance(150.0);
```

The camera pose uses PCL's transposed (row-vector) convention — translation goes in the bottom row of the 4×4 matrix, not the right column. See the inline comments in the source for details.

## Notes

- The filter overwrites the input `.pcd` files in place. Back up your data before running if you want to keep the originals.
- The quaternion in `filter()` is currently a small (~9°) yaw rotation — adjust to match your camera's orientation relative to the LiDAR.
