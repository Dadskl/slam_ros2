#include "orbslam3_dense_ros2/ros2_slam_publisher.h"
#include "orbslam3_dense_ros2/experiment_recorder.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <deque>
#include <cmath>
#include <filesystem>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

class ORBSlamNode : public rclcpp::Node
{
public:
    explicit ORBSlamNode(const std::vector<std::string>& args)
        : Node("orb_slam3_ros2_node")
    {
        const auto share = ament_index_cpp::get_package_share_directory("orbslam3_dense_ros2");
        mode_ = declare_parameter<std::string>("sensor_mode", "rgbd");
        if (mode_ != "rgbd" && mode_ != "stereo" && mode_ != "mono" && mode_ != "stereo_inertial") {
            throw std::invalid_argument("sensor_mode must be rgbd, stereo, mono or stereo_inertial");
        }
        declare_parameter("vocabulary_file", args.size() > 1 ? args[1] : share + "/Vocabulary/ORBvoc.txt.bin");
        declare_parameter("settings_file", args.size() > 2 ? args[2] :
            (mode_ == "rgbd" ? share + "/config/RGB-D/RealSense_D435i.yaml" : std::string()));
        declare_parameter("enable_pangolin", args.size() > 3 && args[3] == "true");
        declare_parameter("rgb_topic", args.size() > 4 ? args[4] : "/camera/camera/color/image_raw");
        declare_parameter("depth_topic", args.size() > 5 ? args[5] : "/camera/camera/aligned_depth_to_color/image_raw");
        declare_parameter("left_topic", "/kitti/left/image_raw");
        declare_parameter("right_topic", "/kitti/right/image_raw");
        declare_parameter("imu_topic", "/kitti/imu");
        declare_parameter("sync_queue_size", 10);
        declare_parameter<std::string>("image_reliability", "best_effort");
        declare_parameter("sync_max_interval", 0.05);
        declare_parameter<std::string>("trajectory_output_prefix", "");
        declare_parameter<std::string>("experiment_output_dir", "results");
        declare_parameter<std::string>("sequence_name", "sequence");
    }

    // Called after shared ownership exists, so the publisher can use this node and its clock.
    void Initialize()
    {
        const auto vocabulary = get_parameter("vocabulary_file").as_string();
        const auto settings = get_parameter("settings_file").as_string();
        for (const auto& file : {vocabulary, settings}) {
            if (!std::filesystem::is_regular_file(file)) {
                throw std::runtime_error("Required file does not exist: " + file);
            }
        }
        const auto queue = get_parameter("sync_queue_size").as_int();
        const auto interval = get_parameter("sync_max_interval").as_double();
        if (queue < 2 || queue > 1000 || !std::isfinite(interval) || interval <= 0.0) {
            throw std::runtime_error("sync_queue_size must be 2..1000 and sync_max_interval must be positive");
        }
        cv::FileStorage config(settings, cv::FileStorage::READ);
        if (!config.isOpened()) throw std::runtime_error("Cannot read camera settings: " + settings);
        if (mode_ == "stereo_inertial") {
            const auto extrinsic = config["IMU.T_b_c1"].mat();
            if (extrinsic.rows != 4 || extrinsic.cols != 4 || extrinsic.type() != CV_32F) {
                throw std::runtime_error("stereo_inertial requires calibrated IMU.T_b_c1 (4x4 float matrix) in settings_file");
            }
            for (const char* key : {"IMU.Frequency", "IMU.NoiseGyro", "IMU.NoiseAcc", "IMU.GyroWalk", "IMU.AccWalk"}) {
                const auto value = config[key];
                if (!value.isReal() || static_cast<double>(value) <= 0.0) {
                    throw std::runtime_error(std::string("Inertial setting must be a positive real number: ") + key);
                }
            }
        }
        rgb_order_ = static_cast<int>(config["Camera.RGB"]) != 0;
        image_width_ = static_cast<int>(config["Camera.width"]);
        image_height_ = static_cast<int>(config["Camera.height"]);
        const auto factor_node = config["File.version"].empty()
            ? config["DepthMapFactor"] : config["RGBD.DepthMapFactor"];
        depth_factor_ = factor_node.empty() ? 1.0 : static_cast<double>(factor_node);
        if (!std::isfinite(depth_factor_) || depth_factor_ <= 0.0) {
            throw std::runtime_error("DepthMapFactor must be positive");
        }
        RCLCPP_INFO(get_logger(), "Loading vocabulary %s and settings %s", vocabulary.c_str(), settings.c_str());
        const auto sensor = mode_ == "stereo_inertial" ? ORB_SLAM3::System::IMU_STEREO
            : mode_ == "stereo" ? ORB_SLAM3::System::STEREO
            : mode_ == "mono" ? ORB_SLAM3::System::MONOCULAR : ORB_SLAM3::System::RGBD;
        slam_system_ = std::make_unique<ORB_SLAM3::System>(vocabulary, settings, sensor,
            get_parameter("enable_pangolin").as_bool());
        ros2_publisher_ = std::make_unique<ORB_SLAM3::ROS2SlamPublisher>(
            slam_system_->GetSlamDataProcessor(), shared_from_this());
        ros2_publisher_->Run();
        std::ostringstream parameter_snapshot;
        for (const auto& name : list_parameters({}, 0).names) {
            const auto parameter = get_parameter(name);
            parameter_snapshot << name << " [" << parameter.get_type_name() << "] = "
                               << parameter.value_to_string() << '\n';
        }
        experiment_.Configure(get_parameter("experiment_output_dir").as_string(),
            get_parameter("sequence_name").as_string(), mode_, settings, parameter_snapshot.str());

        // Live cameras default to best effort; reliable replay can avoid DDS image drops.
        auto image_qos = rclcpp::SensorDataQoS();
        image_qos.keep_last(static_cast<size_t>(queue));
        const auto reliability = get_parameter("image_reliability").as_string();
        if (reliability == "reliable") image_qos.reliable();
        else if (reliability != "best_effort") {
            throw std::invalid_argument("image_reliability must be best_effort or reliable");
        }
        const auto qos = image_qos.get_rmw_qos_profile();
        if (mode_ == "mono") {
            mono_subscriber_ = create_subscription<sensor_msgs::msg::Image>(
                get_parameter("left_topic").as_string(), image_qos,
                std::bind(&ORBSlamNode::MonoCallback, this, std::placeholders::_1));
        } else {
            rgb_subscriber_.subscribe(this,
                get_parameter(mode_ == "rgbd" ? "rgb_topic" : "left_topic").as_string(), qos);
            depth_subscriber_.subscribe(this,
                get_parameter(mode_ == "rgbd" ? "depth_topic" : "right_topic").as_string(), qos);
            sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
                SyncPolicy(static_cast<uint32_t>(queue)), rgb_subscriber_, depth_subscriber_);
            sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(interval));
            if (mode_ == "rgbd") {
                sync_->registerCallback(std::bind(&ORBSlamNode::RgbdCallback, this,
                    std::placeholders::_1, std::placeholders::_2));
            } else if (mode_ == "stereo") {
                sync_->registerCallback(std::bind(&ORBSlamNode::StereoCallback, this,
                    std::placeholders::_1, std::placeholders::_2));
            } else {
                sync_->registerCallback(std::bind(&ORBSlamNode::QueueInertialStereoPair, this,
                    std::placeholders::_1, std::placeholders::_2));
            }
        }
        if (mode_ == "stereo_inertial") {
            imu_subscriber_ = create_subscription<sensor_msgs::msg::Imu>(
                get_parameter("imu_topic").as_string(), rclcpp::SensorDataQoS(),
                std::bind(&ORBSlamNode::ImuCallback, this, std::placeholders::_1));
        }
        RCLCPP_INFO(get_logger(), "%s subscriptions ready", mode_.c_str());
    }

    ~ORBSlamNode() override { Stop(); }

    void Stop()
    {
        if (stopped_) return;
        stopped_ = true;
        sync_.reset();
        mono_subscriber_.reset();
        imu_subscriber_.reset();
        pending_pairs_.clear();
        imu_buffer_.clear();
        rgb_subscriber_.unsubscribe();
        depth_subscriber_.unsubscribe();
        // Join readers before stopping the System-owned data producer.
        ros2_publisher_.reset();
        if (slam_system_) {
            slam_system_->Shutdown();
            try {
                const auto directory = experiment_.Save(*slam_system_);
                if (!directory.empty()) RCLCPP_INFO(get_logger(), "Experiment data saved: %s", directory.c_str());
            } catch (const std::exception& e) {
                RCLCPP_ERROR(get_logger(), "Experiment export incomplete: %s", e.what());
            }
            const auto prefix = get_parameter("trajectory_output_prefix").as_string();
            if (!prefix.empty()) {
                try {
                    const auto parent = std::filesystem::path(prefix).parent_path();
                    if (!parent.empty()) std::filesystem::create_directories(parent);
                    slam_system_->SaveTrajectoryTUM(prefix + "_tum.txt");
                    slam_system_->SaveTrajectoryKITTI(prefix + "_kitti.txt");
                    RCLCPP_INFO(get_logger(), "Saved estimated trajectories: %s_tum.txt and %s_kitti.txt",
                        prefix.c_str(), prefix.c_str());
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(get_logger(), "Cannot save estimated trajectory: %s", e.what());
                }
            }
        }
    }

private:
    void MonoCallback(const sensor_msgs::msg::Image::ConstSharedPtr& image)
    {
        if (slam_system_->isShutDown()) return;
        const size_t input_index = experiment_.Input();
        try {
            if (image->width == 0 || image->height == 0 ||
                (image_width_ > 0 && image->width != static_cast<uint32_t>(image_width_)) ||
                (image_height_ > 0 && image->height != static_cast<uint32_t>(image_height_))) {
                throw std::runtime_error("Monocular image dimensions do not match camera settings");
            }
            const double timestamp = rclcpp::Time(image->header.stamp).seconds();
            if (timestamp <= last_timestamp_) return;
            const auto encoding = rgb_order_ ? sensor_msgs::image_encodings::RGB8 : sensor_msgs::image_encodings::BGR8;
            const auto cv_image = cv_bridge::toCvShare(image, encoding);
            experiment_.Measure(*slam_system_, input_index, timestamp, [&] { slam_system_->TrackMonocular(cv_image->image, timestamp); });
            last_timestamp_ = timestamp;
        } catch (const std::exception& e) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "Mono input: %s", e.what());
        }
    }

    void QueueInertialStereoPair(const sensor_msgs::msg::Image::ConstSharedPtr& left,
                                 const sensor_msgs::msg::Image::ConstSharedPtr& right)
    {
        if (slam_system_->isShutDown()) return;
        const size_t input_index = experiment_.Input();
        const double timestamp = rclcpp::Time(left->header.stamp).seconds();
        const double right_timestamp = rclcpp::Time(right->header.stamp).seconds();
        if (std::abs(timestamp - right_timestamp) > 0.001) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "Stereo-inertial pair timestamps differ by more than 1 ms; pair dropped");
            return;
        }
        if (timestamp <= last_timestamp_) return;
        pending_pairs_.push_back({left, right, input_index});
        if (pending_pairs_.size() > 10) {
            pending_pairs_.pop_front();
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "IMU data is not keeping up with stereo images; dropping oldest image pair");
        }
        DrainInertialStereoPairs();
    }

    void ImuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr& imu)
    {
        const double timestamp = rclcpp::Time(imu->header.stamp).seconds();
        if (!std::isfinite(timestamp) || timestamp <= last_imu_timestamp_) return;
        const auto& a = imu->linear_acceleration;
        const auto& w = imu->angular_velocity;
        if (!std::isfinite(a.x) || !std::isfinite(a.y) || !std::isfinite(a.z) ||
            !std::isfinite(w.x) || !std::isfinite(w.y) || !std::isfinite(w.z)) return;
        imu_buffer_.emplace_back(static_cast<float>(a.x), static_cast<float>(a.y), static_cast<float>(a.z),
            static_cast<float>(w.x), static_cast<float>(w.y), static_cast<float>(w.z), timestamp);
        last_imu_timestamp_ = timestamp;
        while (imu_buffer_.size() > 10000) imu_buffer_.pop_front();
        DrainInertialStereoPairs();
    }

    void DrainInertialStereoPairs()
    {
        while (!pending_pairs_.empty() && !imu_buffer_.empty()) {
            const auto& pair = pending_pairs_.front();
            const double timestamp = rclcpp::Time(pair.first->header.stamp).seconds();
            if (imu_buffer_.back().t < timestamp) break;
            if (timestamp <= last_timestamp_) { pending_pairs_.pop_front(); continue; }
            std::vector<ORB_SLAM3::IMU::Point> measurements;
            while (!imu_buffer_.empty() && imu_buffer_.front().t <= timestamp) {
                if (imu_buffer_.front().t > last_timestamp_) measurements.push_back(imu_buffer_.front());
                imu_buffer_.pop_front();
            }
            if (measurements.empty()) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                    "No IMU measurements for stereo frame; frame dropped");
                pending_pairs_.pop_front();
                continue;
            }
            if (std::isfinite(last_timestamp_) && measurements.size() < 2) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                    "Fewer than two IMU samples between stereo frames; inertial tracking may fail");
            }
            try {
                ValidatePair(*pair.first, *pair.second);
                const auto encoding = rgb_order_ ? sensor_msgs::image_encodings::RGB8 : sensor_msgs::image_encodings::BGR8;
                const auto left = cv_bridge::toCvShare(pair.first, encoding);
                const auto right = cv_bridge::toCvShare(pair.second, encoding);
                experiment_.Measure(*slam_system_, pair.input_index, timestamp, [&] {
                    slam_system_->TrackStereo(left->image, right->image, timestamp, measurements);
                });
                last_timestamp_ = timestamp;
            } catch (const std::exception& e) {
                RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "Stereo-inertial input: %s", e.what());
            }
            pending_pairs_.pop_front();
        }
    }

    void ValidatePair(const sensor_msgs::msg::Image& first, const sensor_msgs::msg::Image& second) const
    {
        if (first.width != second.width || first.height != second.height || first.width == 0 || first.height == 0) {
            throw std::runtime_error("Paired images must have the same nonzero resolution");
        }
        if ((image_width_ > 0 && first.width != static_cast<uint32_t>(image_width_)) ||
            (image_height_ > 0 && first.height != static_cast<uint32_t>(image_height_))) {
            throw std::runtime_error("Image resolution does not match Camera.width/height in settings_file");
        }
    }

    void StereoCallback(const sensor_msgs::msg::Image::ConstSharedPtr& left,
                        const sensor_msgs::msg::Image::ConstSharedPtr& right)
    {
        if (slam_system_->isShutDown()) return;
        const size_t input_index = experiment_.Input();
        try {
            ValidatePair(*left, *right);
            const double timestamp = rclcpp::Time(left->header.stamp).seconds();
            if (timestamp <= last_timestamp_) return;
            const auto encoding = rgb_order_ ? sensor_msgs::image_encodings::RGB8 : sensor_msgs::image_encodings::BGR8;
            const auto left_image = cv_bridge::toCvShare(left, encoding);
            const auto right_image = cv_bridge::toCvShare(right, encoding);
            experiment_.Measure(*slam_system_, input_index, timestamp, [&] { slam_system_->TrackStereo(left_image->image, right_image->image, timestamp); });
            last_timestamp_ = timestamp;
        } catch (const std::exception& e) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "Stereo input: %s", e.what());
        }
    }

    void RgbdCallback(const sensor_msgs::msg::Image::ConstSharedPtr& rgb,
                      const sensor_msgs::msg::Image::ConstSharedPtr& depth)
    {
        if (slam_system_->isShutDown()) return;
        const size_t input_index = experiment_.Input();
        try {
            ValidatePair(*rgb, *depth);
            const auto color = cv_bridge::toCvShare(rgb, rgb_order_
                ? sensor_msgs::image_encodings::RGB8 : sensor_msgs::image_encodings::BGR8);
            const auto depth_image = cv_bridge::toCvShare(depth);
            cv::Mat input_depth;
            // REP-118: 16UC1 is millimetres, 32FC1 is metres. ORB divides by the YAML factor.
            if (depth->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
                depth_image->image.convertTo(input_depth, CV_32F, depth_factor_ / 1000.0);
            } else if (depth->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
                depth_image->image.convertTo(input_depth, CV_32F, depth_factor_);
            } else {
                throw std::runtime_error("Depth encoding must be 16UC1 (mm) or 32FC1 (m)");
            }
            // ORB treats positive depth as valid; infinity must not create invalid map points.
            for (int row = 0; row < input_depth.rows; ++row) {
                auto* values = input_depth.ptr<float>(row);
                for (int col = 0; col < input_depth.cols; ++col) {
                    if (!std::isfinite(values[col]) || values[col] <= 0.0f) values[col] = 0.0f;
                }
            }
            const double timestamp = rclcpp::Time(rgb->header.stamp).seconds();
            if (timestamp <= last_timestamp_) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                    "Ignoring non-increasing image timestamp; restart SLAM when replaying a bag");
                return;
            }
            experiment_.Measure(*slam_system_, input_index, timestamp, [&] { slam_system_->TrackRGBD(color->image, input_depth, timestamp); });
            last_timestamp_ = timestamp;
        } catch (const std::exception& e) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "RGB-D input: %s", e.what());
        }
    }

    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
    std::unique_ptr<ORB_SLAM3::System> slam_system_;
    std::unique_ptr<ORB_SLAM3::ROS2SlamPublisher> ros2_publisher_;
    message_filters::Subscriber<sensor_msgs::msg::Image> rgb_subscriber_, depth_subscriber_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mono_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    struct PendingPair {
        sensor_msgs::msg::Image::ConstSharedPtr first, second;
        size_t input_index;
    };
    std::deque<PendingPair> pending_pairs_;
    ORB_SLAM3::ExperimentRecorder experiment_;
    std::deque<ORB_SLAM3::IMU::Point> imu_buffer_;
    bool rgb_order_ = false;
    std::string mode_;
    int image_width_ = 0, image_height_ = 0;
    double depth_factor_ = 1.0;
    double last_timestamp_ = -std::numeric_limits<double>::infinity();
    double last_imu_timestamp_ = -std::numeric_limits<double>::infinity();
    bool stopped_ = false;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    int result = 0;
    std::shared_ptr<ORBSlamNode> node;
    try {
        node = std::make_shared<ORBSlamNode>(rclcpp::remove_ros_arguments(argc, argv));
        node->Initialize();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("orb_slam3_main"), "%s", e.what());
        result = 1;
    }
    if (node) node->Stop();
    node.reset();
    if (rclcpp::ok()) rclcpp::shutdown();
    return result;
}
