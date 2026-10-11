# 用 KITTI odometry 彩色双目图像运行 SLAM

本项目 ROS 入口提供 `rgbd`、`mono`、`stereo` 和 `stereo_inertial`。当前 KITTI odometry 彩色图像可以用于 `mono`（只用 `image_2`）或 `stereo`（`image_2`＋`image_3`）。双目惯性需要额外的同步 IMU 原始测量；Velodyne 激光点云和 `poses` 真值都不能替代 IMU。

双目模式输出有尺度的相机轨迹和稀疏特征点地图。`cloudPointMapping` 稠密点云仍属于 RGB-D 功能：仅发送左右彩色图片不会生成深度图。若要做 KITTI 稠密重建，需要另加双目深度估计，并调整原 RGB-D 点云代码的 5 米距离上限。

## 数据目录

```text
dataset/sequences/00/
├── image_2/         # 左彩色图：000000.png, 000001.png, ...
├── image_3/         # 右彩色图：与左图帧号一一对应
├── calib.txt        # 需要 P2、P3
└── times.txt        # 每帧对应一行时间，单位秒
```

图片须为 KITTI odometry 提供的已校正双目图。不同 KITTI 数据集（raw、object detection、KITTI-360）的目录和标定格式不属于此脚本输入。若没有 `calib.txt` 或 `times.txt`，请补齐对应的 odometry 标定/时间文件。

## 先生成该序列的配置

先将独立 `kitti_dataset_tools` 包放在工作空间 `src/` 下（与本 SLAM 包同级），在工作空间根目录运行 `colcon build --packages-select kitti_dataset_tools`。它不依赖 ORB-SLAM3，数据集整理、配置生成与播放的实现均在该包中。然后执行：

```bash
source /opt/ros/jazzy/setup.bash
source ~/robot_ws/install/setup.bash

ros2 run kitti_dataset_tools kitti_player \
  --sequence-dir /你的数据目录/dataset/sequences/00 \
  --settings-output ~/robot_ws/kitti_00_color.yaml \
  --prepare-only
```

脚本从 `P2/P3` 提取彩色相机内参和左右相机中心之间的基线，从图片读取分辨率，从 `times.txt` 推算帧率。不要把 `P0/P1` 灰度相机标定或其他序列的 YAML 直接用于彩色图片。脚本使用 OpenCV 读取图片，因此发布编码为 `bgr8`，生成的配置也设置 `Camera.RGB: 0`。

## 再启动 SLAM 和发布器

终端一：

```bash
source ~/robot_ws/install/setup.bash
ros2 launch orbslam3_dense_ros2 stereo.launch.py \
  settings_file:=$HOME/robot_ws/kitti_00_color.yaml \
  use_sim_time:=true \
  trajectory_output_prefix:=$HOME/robot_ws/results/00
```

终端二：

```bash
source ~/robot_ws/install/setup.bash
ros2 run kitti_dataset_tools kitti_player \
  --sequence-dir /你的数据目录/dataset/sequences/00 \
  --publish-clock --speed 1.0 --reliability reliable
```

发布器默认等待左右话题都出现订阅者后再开始，最多等 60 秒。它逐帧读取图片，不会把全部序列读入内存。`--speed 0.5` 表示半速回放；`--start-frame 100 --max-frames 500` 只播放指定片段。回放速度改变发送节奏，不改变原始图像时间戳。

使用 `--publish-clock` 时，图像和 `/clock` 都使用数据集时间，SLAM 应设置 `use_sim_time:=true`。不加此选项时，图像时间戳以当前系统时间为起点，但保留数据集时间间隔，此时 SLAM 保持默认 `use_sim_time:=false`。不要同时运行多个 `/clock` 发布器。重新播放序列前，应重启 SLAM。

发布器只播放一次，结束后自行退出。等待最后一帧处理完，再对 SLAM 按 `Ctrl+C`；退出时会生成 `00_tum.txt`（时间戳、位置、四元数）和 `00_kitti.txt`（每行 3×4 位姿矩阵）。评估时优先使用 TUM 文件的时间戳去匹配 `times.txt` 的帧序号，再与 `poses/00.txt` 对齐。跟踪失败的帧没有估计位姿，KITTI 文件因此不保证逐行对应原始帧号。导出的位姿采用 SLAM 相机坐标系，官方真值和它可能有固定外参差异；需要先做坐标系与起点对齐，不能直接逐行相减。

## 单目和双目惯性模式

单目模式只接收 `image_2`。使用上面生成的同一份相机 YAML（只读取左相机的内参），启动 `mono.launch.py`，并给播放脚本增加 `--mode mono`。单目轨迹的全局尺度无法由图像单独确定；与真值比较时需使用 Sim(3) 对齐，并把是否对齐尺度写入评测结果。

```bash
ros2 launch orbslam3_dense_ros2 mono.launch.py settings_file:=$HOME/robot_ws/kitti_00_color.yaml use_sim_time:=true trajectory_output_prefix:=$HOME/robot_ws/results/00_mono
ros2 run kitti_dataset_tools kitti_player --sequence-dir /你的数据目录/dataset/sequences/00 --mode mono --publish-clock --reliability reliable
```

两条命令分别在两个终端运行。双目惯性模式使用 `stereo_inertial.launch.py`，它要求左右图像和 `sensor_msgs/msg/Imu` 具有同一时间基准。订阅端默认接收 `/kitti/imu`，缓冲并按帧时间戳向 ORB-SLAM3 提交测量。配置必须包含实测 `IMU.T_b_c1`、`IMU.Frequency`、`IMU.NoiseGyro`、`IMU.NoiseAcc`、`IMU.GyroWalk`、`IMU.AccWalk`。当前 `kitti_player` **不会发布 IMU**，因为 odometry 图像、Velodyne、标定和位姿压缩包都没有 IMU 原始测量。不能凭 `poses` 合成后宣称是实测 IMU。

KITTI Raw 的 `*_sync` 版本包含与图像同步、约 10 Hz 的 OXTS GPS/IMU 记录和相机—IMU 标定；要获得原始高频测量，还要取得同一 drive 的 `*_extract` OXTS 及其原始时间戳。二者是独立于 odometry 压缩包的数据产品。准备使用时，须先确认 odometry 序列到 Raw drive 的映射、各自时间戳、相机—IMU 外参和噪声参数，再编写对应发布器；不能直接按 odometry 帧号拼接 `*_extract` OXTS。

## 订阅接口要求

| 项目 | 双目模式 |
| --- | --- |
| 左话题 | `/kitti/left/image_raw`，可通过 `left_topic` 改名 |
| 右话题 | `/kitti/right/image_raw`，可通过 `right_topic` 改名 |
| 消息类型 | 两路都是 `sensor_msgs/msg/Image`，不是 `CompressedImage` |
| 图像编码 | 脚本发布 `bgr8`；节点也能转换 `rgb8` 等 cv_bridge 支持的图像编码 |
| 同步 | 同一帧左右图像使用完全相同的 `header.stamp` |
| 图像尺寸 | 左右一致，并且与 YAML 的 `Camera.width/height` 一致 |
| 标定 | 从 YAML 读取，不订阅 `CameraInfo` |
| QoS | 节点使用 SensorDataQoS（best effort）；脚本默认 reliable，可兼容 reliable 或 best effort 订阅端 |
| 时间顺序 | 时间戳严格递增；节点丢弃重复或倒退的时间戳 |

节点内部通过 `System::STEREO` 创建 SLAM 系统，并将同步图像送入 `TrackStereo()`。只修改话题名字，无法让 RGB-D 模式自动变成双目模式；使用提供的 `stereo.launch.py` 会正确设置模式。

## 查看结果

```bash
ros2 topic hz /kitti/left/image_raw
ros2 topic hz /kitti/right/image_raw
ros2 topic echo /camera_pose --once
```

在 RViz 中将 Fixed Frame 设为 `map`，添加 Path `/camera_path`，以及 PointCloud2 `/point_cloud_all`、`/point_cloud_ref`。这里的 `map` 保留项目的“前、左、上”地面坐标约定；若要与 KITTI 官方相机坐标系下的真值轨迹比较，还需要做坐标系转换，不能直接比较消息的原始 XYZ。

相机外参之外，项目还保留了原先示例车辆安装外参。KITTI 实验优先查看 `camera_pose`、`camera_path`，不要将 `vehicle_pose` 当作 KITTI 官方车辆坐标系真值。

默认不打开 Pangolin。需要桌面窗口时，在终端一的 launch 命令加 `enable_pangolin:=true`。使用 `Ctrl+C` 退出 SLAM。

来源：[KITTI odometry 官方页面](https://www.cvlibs.net/datasets/kitti/eval_odometry.php)、[官方页面推荐的 pykitti 标定和时间读取实现](https://github.com/utiasSTARS/pykitti/blob/master/pykitti/odometry.py)。

## 同步发布 Velodyne 激光点云

在原有 `kitti_player` 命令后追加 `--publish-velodyne`，读取当前序列的
`velodyne/000000.bin` 等文件，与同帧图像使用相同 ROS 时间戳。默认不启用。

```bash
ros2 run kitti_dataset_tools kitti_player \
  --dataset-root /home/alabex/dataset/kitti --sequence 00 \
  --publish-velodyne --publish-clock --wait-for-subscribers 0
```

上面的命令直接播放，适合查看 topic。运行 SLAM 时，先启动原有 stereo launch，
设置 `use_sim_time:=true`，去掉 `--wait-for-subscribers 0` 即可等待图像订阅者。
播放器不要求激光订阅者在线。

- 默认话题 `/kitti/velodyne_points`，类型 `sensor_msgs/msg/PointCloud2`。
- 字段 `x/y/z/intensity` 均为 float32，每点 16 字节，保留 KITTI 反射强度。
- 默认 frame 为 `kitti_velodyne`，原始雷达坐标，单位米，x 前、y 左、z 上。
- `--velodyne-topic` 和 `--velodyne-frame` 可修改话题和帧名；改帧名不会变换坐标。
- 点云和图像共用 `--speed`、`--start-frame`、`--max-frames`、`--publish-clock`
  和 `--reliability`（默认 reliable、volatile、depth 10）。
- 启动时检查整段序列点云与图像帧号一一对应、文件非空且长度为 16 的倍数。
  缺失或损坏会报错，不会静默跳帧；`--prepare-only` 也执行该检查。

```bash
ros2 topic info /kitti/velodyne_points
ros2 topic hz /kitti/velodyne_points
```

RViz 查看原始点云时，将 Fixed Frame 设为 `kitti_velodyne`，添加 PointCloud2
并选择该话题。这次仅增加数据发布，不发布相机与雷达之间的静态 TF，不进行运动补偿，
也未将点云接入 ORB-SLAM3 位姿估计或全局建图。因此不能直接在 `map/odom`
下叠加显示；后续融合需使用标定外参，并正确处理左彩色相机坐标系。

修改源码后需重新构建并 source 工作空间的 `install/setup.bash`。
