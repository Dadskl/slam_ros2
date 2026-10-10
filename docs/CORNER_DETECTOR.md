# 角点检测接口修改报告

## 修改目的

将候选角点检测从 ORB 特征提取流程中抽离，允许在构造提取器时注入其他检测算法。默认仍使用 FAST，保留原有金字塔、空间分配、方向计算与 ORB 描述子计算。

## 修改文件

### `orb_slam3/include/ORBextractor.h`

- 增加抽象类 `CornerDetector`，提供 `Detect(image, keypoints, relaxed)` 接口。
- 增加默认实现 `FastCornerDetector`，保存初始阈值和备用阈值。
- `ORBextractor` 构造函数增加可选的第六个参数：`std::unique_ptr<CornerDetector>`。
- 增加检测器所有权成员；省略第六个参数或传入空指针时自动创建 FAST。
- 原有五参数调用无需修改。由于使用独占所有权，提取器不能再隐式复制；项目现有调用不复制提取器。类布局和构造函数签名发生变化，使用该头文件的二进制需要重新编译。

### `orb_slam3/src/ORBextractor.cc`

- 实现 `FastCornerDetector`，继续调用启用非极大值抑制的 `cv::FAST`。
- 构造时接收自定义检测器，或创建默认 FAST。
- `ComputeKeyPointsOctTree()` 中的直接 FAST 调用改为统一接口；候选点为空时使用 `relaxed=true` 重试。
- 保留的 `ComputeKeyPointsOld()` 同样改为统一接口，继续维持原有“不超过三个点时重试”的条件。
- 删除替换区域中已注释的历史 FAST 分支，保留原有坐标偏移和后处理。

### `docs/CORNER_DETECTOR.md`

本修改报告及接入说明。

## 接口约定

```cpp
virtual void Detect(const cv::Mat& image,
                    std::vector<cv::KeyPoint>& keypoints,
                    bool relaxed = false) const = 0;
```

- 输入为金字塔某层的一块 `CV_8UC1` 灰度图，可能不是连续内存；不要修改输入。
- 每次调用替换输出，而不是向上次结果追加。
- 输出 `pt` 为分块内部坐标，须有限且位于图像内；不要加分块偏移或换算回原始图像。
- 输出有效、有限的 `response`，数值越大代表质量越好；空间分配依赖此评分。
- `relaxed=true` 表示算法自行放宽检测条件，例如 FAST 降低阈值，Shi–Tomasi 降低质量阈值。
- 提取器负责坐标偏移、空间分配、octave、size、方向和描述子。自定义检测器仍需遵循接口约定；接口不自动修复无效坐标或评分。
- 每个提取器独立拥有检测器，不提供运行过程中热切换接口。

## 如何接入另一个算法

下面是可编译的 AGAST 示例，仅用于说明接入方式；当前默认算法仍是 FAST。

```cpp
#include "ORBextractor.h"
#include <memory>

class AgastCornerDetector final : public ORB_SLAM3::CornerDetector {
public:
    AgastCornerDetector(int initial, int minimum)
        : initial_(initial), minimum_(minimum) {}

    void Detect(const cv::Mat& image,
                std::vector<cv::KeyPoint>& keypoints,
                bool relaxed = false) const override {
        keypoints.clear();
        cv::AGAST(image, keypoints,
                  relaxed ? minimum_ : initial_, true);
    }

private:
    int initial_;
    int minimum_;
};

// 放在创建提取器的位置，下面数值仅为示例。
// ORB_SLAM3::ORBextractor extractor(
//     1000, 1.2f, 8, 20, 7,
//     std::make_unique<AgastCornerDetector>(20, 7));
```

真正让 SLAM 使用新方法，需要在 `Tracking.cc` 创建 `ORBextractor` 时传入自定义实例。检查 `new ORBextractor` 的全部位置：当前有两条配置读取路径，每条路径均包含左图、右图和单目初始化提取器。每个调用用 `make_unique` 创建独立实例，保持原有特征数量等参数。

当前未添加 YAML/ROS 算法选择参数；选择入口是构造函数第六个参数。之后可增加工厂函数统一根据配置创建检测器。若将新算法实现放进单独的 `.cc` 文件，需要把该文件加入根目录 CMakeLists.txt 的 `orb_slam3_lib` 源文件列表。

## 验证

- 用修改前保存的源码和修改后的源码编译独立对照程序，在空白、低对比度随机纹理、随机纹理、模糊随机纹理四种 640×480 图像上比较；角点数量分别为 0、584、1003、1003。
- 四组角点的位置、response、方向、octave、size、返回的非重叠区域计数以及描述子逐项完全一致。
- 注入自定义探针检测器，验证正常检测与 relaxed 备用检测均被调用，并验证无角点输出能正常处理。
- 文档中的 AGAST 接入示例通过 C++17 语法检查。
- 上述对照程序放在临时目录，未增加项目构建依赖。
- 完整项目构建通过：`cmake --build /home/alabex/ros2_slam_ws/build/orbslam3_dense_ros2 -j2`，核心库、ROS 发布库和 `orb_slam3_main` 均成功编译链接。构建日志包含第三方 Eigen 弃用等警告。
- 未运行真实数据集轨迹评估或 ROS 集成测试；未执行安装，`install/` 中的运行版本未更新。

此次接口重构不代表已经验证新检测算法的定位精度；更换算法后仍需对同一数据集比较轨迹误差、跟踪成功率与耗时。
