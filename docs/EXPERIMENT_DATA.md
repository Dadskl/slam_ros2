# 实验数据导出与校验

SLAM 节点正常退出（例如 Ctrl+C）时，默认在工作目录的 `results/` 下创建一次运行的独立目录。数据集播放器结束不会自动停止 SLAM。不要用 `kill -9` 代替正常退出。

所有 launch 均支持：

```bash
ros2 launch orbslam3_dense_ros2 stereo.launch.py \
  settings_file:=/绝对路径/kitti_00.yaml \
  sequence_name:=KITTI_00 \
  experiment_output_dir:=/绝对路径/results \
  image_reliability:=reliable \
  use_sim_time:=true
```

离线 KITTI 播放器默认使用 reliable 发布。离线实验建议 SLAM 同样使用 `image_reliability:=reliable`；默认 best_effort 仍用于兼容实时相机。可靠订阅无法连接仅提供 best_effort 的相机发布器。可靠传输也不代表任意播放速度下都能完整处理，应核对实际处理帧数与计划播放帧数。

## 目录与数据

```text
<sequence_name>_<UTC开始时间>/
├── trajectories/
│   ├── map_<id>_camera_tum.txt
│   ├── map_<id>_camera_kitti.txt
│   ├── map_<id>_keyframes_tum.txt
│   └── camera_tum.txt / camera_kitti.txt  # 仅单个非空当前地图时提供
├── maps/
│   ├── map_<id>_sparse.ply
│   └── index.csv
├── statistics/
│   ├── frames.csv
│   ├── tracking_losses.csv
│   └── summary.txt
├── config/
│   ├── camera.yaml
│   ├── parameters.txt
│   └── build_revision.txt
└── README.txt
```

文件中的 `map_id` 表示运行中的地图标识。地图合并、清除或重置后，最终保留的地图及轨迹可能少于在线统计中的有效帧。不同独立地图不可直接拼成同一个世界坐标系；每张地图的点云、相机轨迹、关键帧轨迹均以它的首个有效关键帧为原点。单目结果没有已知的绝对尺度。

`frames.csv` 的 `input_index` 是节点收到的单目回调/同步帧对序号，不是数据集原始帧号。同步前和 DDS 传输中的丢帧无法由这个序号反映。TUM 时间戳可与数据集 `times.txt` 对照。`completed_tracking_calls` 和 `valid_pose_ratio_received` 不代表整个数据集的完成率。

状态 2（OK）或 5（OK_KLT）且已经赋值位姿才标记为有效。状态 3（RECENTLY_LOST）的预测位姿不作为有效测量导出。丢失片段从状态 3/4 开始，包含随后的重初始化，一直持续到再次获得有效位姿；启动初始化不计入丢失。`recovered=0` 表示截至停止时仍未恢复，持续时间仅统计到最后一个观测时间。

`tracking_ms` 是同步 Track 调用的墙钟耗时，不包括 ROS 图像转换、实验记录和后台线程的完整 CPU 开销。左右特征提取耗时分别记录，双目并行执行时不能将它们相加当作总耗时。`map_inlier_associations` 是当前帧非外点的持久地图点关联数，不是优化器内部计数，也不是去重后的地图点数。

配置在初始化时捕获。`build_revision.txt` 保存 Git 描述及源码 SHA-256 指纹；没有提交记录的工作目录也可以通过源码指纹区分。该指纹覆盖构建配置、主要源码、第三方源码和启动/播放脚本，不是外部系统依赖或词典内容的校验和。

## 回归测试

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_slam_ws/install/setup.bash
cd ~/ros2_slam_ws
colcon test --packages-select orbslam3_dense_ros2 --event-handlers console_direct+
colcon test-result --verbose
```

自动测试检查两种 RGB-D 深度编码、双目、空输入、空白初始化、跟踪丢失与恢复、重复运行目录不覆盖，以及输出数据的一致性。`test/experiment_assertions.py` 会重新计算统计汇总，并检查时间戳、四元数、KITTI 旋转矩阵、地图点数、坐标原点和轨迹有效性。这些检查不能代替基于真值的 ATE/RPE 精度评估。
