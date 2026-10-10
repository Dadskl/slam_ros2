# ORB-SLAM3 dense mapping for ROS 2 Jazzy

[中文安装与使用说明](README_CN.md)

Target: **Ubuntu 24.04, ROS 2 Jazzy, C++17**, distribution OpenCV 4 / Eigen 3 / PCL, and Pangolin 0.9.2. CUDA is not required. The ROS executable supports aligned RGB-D and rectified stereo images. See the [KITTI color-stereo guide](docs/KITTI.md) for dataset playback. Stereo produces trajectories and sparse maps; dense clouds still require depth images.

## Build

Install [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html), place this repository in `~/robot_ws/src/orbslam3_dense_ros2`, then run:

```bash
sudo apt update
sudo apt install build-essential cmake git python3-colcon-common-extensions python3-rosdep \
  libeigen3-dev libopencv-dev libpcl-dev libboost-serialization-dev libssl-dev \
  libglew-dev libepoxy-dev libgl1-mesa-dev libegl1-mesa-dev libx11-dev \
  libwayland-dev libxkbcommon-dev
cd ~/robot_ws/src/orbslam3_dense_ros2
bash scripts/install_pangolin.sh "$HOME/.local"
source /opt/ros/jazzy/setup.bash
export CMAKE_PREFIX_PATH="$HOME/.local${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
cd ~/robot_ws
# Run sudo rosdep init first if rosdep has never been initialized.
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y
MAKEFLAGS="-j2 -l2" CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --packages-select orbslam3_dense_ros2 \
  --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select orbslam3_dense_ros2 --event-handlers console_direct+
colcon test-result --verbose
```

Use one build job on machines with limited memory. Avoid mixing a custom OpenCV with the distribution `cv_bridge`. DBoW2 and g2o are built automatically. Launch files, settings and the vocabulary are installed with the package.

## Run

```bash
source ~/robot_ws/install/setup.bash
ros2 launch orbslam3_dense_ros2 rgbd.launch.py \
  settings_file:=/absolute/path/to/your-camera.yaml \
  rgb_topic:=/camera/camera/color/image_raw \
  depth_topic:=/camera/camera/aligned_depth_to_color/image_raw
```

The installed D435i configuration is an example: replace the intrinsics, distortion, image size and frame rate with your camera calibration. RGB and depth must have the same resolution and depth must be aligned to RGB. Depth encodings follow [REP-118](https://www.ros.org/reps/rep-0118.html): `16UC1` millimetres or `32FC1` metres. The node adapts to the YAML depth scale and color order. Best-effort subscriptions also accept reliable image publishers.

Pangolin is disabled by default for headless operation; use `enable_pangolin:=true` on a desktop. For bag playback set `use_sim_time:=true` and play with `--clock`. Restart SLAM before replaying timestamps that go backwards. The node supports ROS parameters and the legacy positional arguments `vocabulary settings viewer rgb_topic depth_topic`.

Outputs: `camera_pose`, `vehicle_pose`, `camera_path`, `vehicle_path`, `point_cloud_all`, `point_cloud_ref`, `cloudPointMapping`, `current_frame`. In RViz select fixed frame `odom` and PointCloud2 topic `cloudPointMapping`.

The original mounting transforms are retained: the camera/vehicle translation in `SlamDataProcess.cc` is `(0.1, 0, 0.065)` and the publisher adds a static `map → odom` height offset of `0.065 m`. Adjust these to your actual mounting. Set `ground_frame:=map` to omit the extra static transform. See the Chinese guide for the complete parameter table.

## Reproducible validation

```bash
docker build --progress=plain -f docker/Dockerfile.jazzy -t orbslam3-jazzy .
```

The Dockerfile builds and runs tests against official Jazzy packages. GitHub Actions uses the same image recipe. Synthetic RGB-D tests cover startup, empty frames, both depth encodings, sensor QoS, pose/cloud/image/path output, metric depth, bounded trajectory history, and clean SIGINT shutdown. Real camera accuracy and desktop OpenGL operation require separate validation.

Based on [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3), [ros2_orb_slam3](https://github.com/Mechazo11/ros2_orb_slam3) and [Map_ORBSLAM_ROS](https://github.com/LeonardoDiCaprio1/Map_ORBSLAM_ROS).
