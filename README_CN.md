# ORB-SLAM3 稠密建图 — ROS 2 Jazzy

目标环境为 **Ubuntu 24.04、ROS 2 Jazzy、C++17**。使用系统 OpenCV 4、Eigen 3、PCL 和源码安装的 Pangolin 0.9.2。ROS 入口支持 RGB-D、单目、双目和双目惯性模式；KITTI 彩色图像使用方法见 [KITTI 指南](docs/KITTI.md)。稠密点云仍需要深度图。双目惯性模式需要同步的 IMU 原始测量以及实测的相机—IMU 标定；当前 KITTI odometry 压缩包不包含这些测量。

## 编译前自动检查

在项目根目录执行：

```bash
bash scripts/check_build_env.sh
```

脚本要求 **Ubuntu 24.04 LTS + ROS 2 Jazzy**。系统或 ROS 版本不匹配、Jazzy 未安装或环境混用了其他 ROS 时，会说明原因并退出，不自动升级系统或安装其他 ROS 版本。尚未加载环境时会自动加载 `/opt/ros/jazzy/setup.bash`。

版本通过后，脚本列出并自动通过 apt 安装缺少的系统、ROS 和测试依赖；可能需要输入 sudo 密码。Pangolin 会用无图形窗口的小程序验证编译、链接和动态库加载；不可用时调用 `scripts/install_pangolin.sh` 构建 v0.9.2，默认安装到 `~/.local`。最后验证本项目 CMake 配置，通过才显示 **“可以编译”**。这些检查不会编译完整 SLAM 项目。

```bash
# 只检查，不安装依赖或构建安装 Pangolin（仍会编译小型验证程序）
bash scripts/check_build_env.sh --check-only

# 自定义 Pangolin 安装位置及构建并行数
bash scripts/check_build_env.sh --pangolin-prefix "$HOME/.local" --jobs 2
```

检查日志默认位于 `~/.cache/orbslam3_dense_ros2/environment/` 的独立运行目录，可用 `--log-dir` 指定。通过后按提示 `source` 生成的 `build_env.sh`，再在工作空间根目录执行它给出的 `colcon build` 命令。脚本不会自动改写 `.bashrc`。不要在已激活的 Conda/virtualenv 中进行此项目的 ROS 编译。

## 安装与编译

先按 [ROS 官方指南](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) 安装 Jazzy，然后在 Ubuntu 终端执行：

```bash
sudo apt update
sudo apt install build-essential cmake git python3-colcon-common-extensions python3-rosdep \
  libeigen3-dev libopencv-dev libpcl-dev libboost-serialization-dev libssl-dev \
  libglew-dev libepoxy-dev libgl1-mesa-dev libegl1-mesa-dev libx11-dev \
  libwayland-dev libxkbcommon-dev

mkdir -p ~/robot_ws/src
cd ~/robot_ws/src
git clone https://github.com/zgfbupt/orbslam3_dense_ros2.git
cd orbslam3_dense_ros2
bash scripts/install_pangolin.sh "$HOME/.local"

source /opt/ros/jazzy/setup.bash
export CMAKE_PREFIX_PATH="$HOME/.local${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
cd ~/robot_ws
# 仅首次使用 rosdep 时执行 sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y
MAKEFLAGS="-j2 -l2" CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --packages-select orbslam3_dense_ros2 \
  --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select orbslam3_dense_ros2 --event-handlers console_direct+
colcon test-result --verbose
```

如果你已下载或修改此项目，将当前目录放入 `~/robot_ws/src/`，跳过重新克隆。不要与旧工作空间的 `build/install/log` 混用。ORB-SLAM3 的优化器编译占用较多内存；内存较小时将并行数设为 `1`。不建议用自行安装到 `/usr/local` 的另一版 OpenCV 覆盖系统版本，以免与 Jazzy 的 `cv_bridge` 混用不同 ABI。

第三方 DBoW2、g2o 由同一次构建生成和安装，无需预先编译。词典、相机配置和 launch 文件也会安装到包的 share 目录，不再依赖开发者机器上的绝对路径。

## 启动

```bash
source /opt/ros/jazzy/setup.bash
source ~/robot_ws/install/setup.bash
ros2 launch orbslam3_dense_ros2 rgbd.launch.py \
  settings_file:=/绝对路径/你的相机.yaml \
  rgb_topic:=/camera/camera/color/image_raw \
  depth_topic:=/camera/camera/aligned_depth_to_color/image_raw
```

也可以直接运行：

```bash
ros2 run orbslam3_dense_ros2 orb_slam3_main --ros-args \
  -p settings_file:=/绝对路径/你的相机.yaml -p enable_pangolin:=false
```

默认加载随包安装的 D435i **示例**配置和二进制词典。示例内参不是你的相机标定值；请复制配置，填写实际分辨率、RGB 相机内参、畸变和帧率。深度必须对齐到 RGB 图像，并且两者分辨率一致。

`16UC1` 深度按毫米、`32FC1` 深度按米解释，节点会匹配 YAML 的深度比例；这是 [REP-118](https://www.ros.org/reps/rep-0118.html) 的约定。RGB 图像会按 YAML 的 `Camera.RGB` 自动选择 RGB/BGR 转换。传感器订阅采用 best-effort QoS，可接收 best-effort 或 reliable 相机流。

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `vocabulary_file` | 已安装的 `ORBvoc.txt.bin` | 支持此项目的二进制格式或 ORB 文本词典 `.txt` |
| `settings_file` | 已安装的 D435i 配置 | 应替换成自己的标定配置 |
| `enable_pangolin` | `false` | 无显示器可运行；有桌面环境时设为 `true` |
| `sync_queue_size` | `10` | RGB-D 同步缓冲，范围 2–1000 |
| `sync_max_interval` | `0.05` | 允许的 RGB-D 时间差，单位秒 |
| `publish_rate_hz` | `30.0` | 发布频率，范围 1–120 Hz |
| `use_sim_time` | `false` | 回放 rosbag 时设为 `true`，并用 `ros2 bag play ... --clock` |
| `ground_frame` | `odom` | 输出的世界坐标系 |
| `camera_frame` | `camera_color_optical_frame`（launch） | 可视化图像坐标系 |
| `vehicle_frame` | `base_footprint` | 车辆位姿 TF 的子坐标系 |

回放时间倒退时请重新启动 SLAM；节点会丢弃重复或倒退时间戳。`Ctrl+C` 会停止并等待发布和 SLAM 工作线程退出。

## 输出与坐标系

输出话题包括 `camera_pose`、`vehicle_pose`、`camera_path`、`vehicle_path`、`point_cloud_all`、`point_cloud_ref`、`cloudPointMapping` 和 `current_frame`。在 RViz 中添加 PointCloud2，选择 `cloudPointMapping`，固定坐标系设为 `odom`。

项目保留了原来的光学坐标系到地面坐标系旋转，以及车辆安装外参：`SlamDataProcess.cc` 中相机与车辆的平移为 `(0.1, 0, 0.065)`；发布器还保留 `map → odom` 的静态高度偏移 `0.065 m`。用于实际车辆前必须按安装位置调整这两处；不能把示例外参当作标定结果。将 `ground_frame:=map` 可取消额外的 `map → ground_frame` 静态变换。

## 可重复验证

```bash
docker build --progress=plain -f docker/Dockerfile.jazzy -t orbslam3-jazzy .
```

Docker 构建使用官方 Jazzy 镜像，并执行 `colcon build` 和运行测试。GitHub Actions 使用同一个 Dockerfile。测试以合成 RGB-D 图像检查无显示器启动、空白帧初始化、两种深度编码、best-effort 订阅、位姿/图像/点云/轨迹输出、两米点云尺度、轨迹不重复增长和 SIGINT 正常退出。它不替代真实相机标定、真实场景精度评估或桌面 OpenGL 验证。

## 参考

- [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3)
- [ros2_orb_slam3](https://github.com/Mechazo11/ros2_orb_slam3)
- [Map_ORBSLAM_ROS](https://github.com/LeonardoDiCaprio1/Map_ORBSLAM_ROS)
- [原项目演示视频](https://www.bilibili.com/video/BV1g8Y9zMEZ4/)
