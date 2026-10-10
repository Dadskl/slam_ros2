#ifndef ORBSLAM3_EXPERIMENT_RECORDER_H
#define ORBSLAM3_EXPERIMENT_RECORDER_H

#include "System.h"
#include "Tracking.h"
#include "Atlas.h"
#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <stdexcept>
#include <sstream>
#include <string>
#include <vector>

#ifndef ORBSLAM3_BUILD_REVISION
#define ORBSLAM3_BUILD_REVISION "unknown"
#endif
#ifndef ORBSLAM3_SOURCE_SHA256
#define ORBSLAM3_SOURCE_SHA256 "unknown"
#endif

namespace ORB_SLAM3 {

// Called on the node's single executor thread. Final export requires System::Shutdown().
class ExperimentRecorder {
public:
    using Clock = std::chrono::steady_clock;
    struct Sample {
        size_t input_index;
        double timestamp, tracking_ms, left_ms, right_ms;
        int state;
        bool valid;
        size_t features_left, features_right, inliers;
        unsigned long map_id;
    };

    void Configure(const std::string& root, const std::string& sequence,
                   const std::string& mode, const std::string& settings,
                   const std::string& parameters) {
        if (root.empty()) throw std::invalid_argument("experiment_output_dir must not be empty");
        root_ = std::filesystem::absolute(root).string();
        mode_ = mode;
        settings_path_ = settings;
        parameters_ = parameters;
        std::ifstream input(settings, std::ios::binary);
        if (!input) throw std::runtime_error("Cannot snapshot camera settings: " + settings);
        settings_contents_.assign(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());
        std::string safe;
        for (unsigned char c : sequence) {
            safe += ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
                     (c >= '0' && c <= '9') || c == '-' || c == '_') ? char(c) : '_';
        }
        if (safe.empty()) safe = "sequence";
        const auto now = std::chrono::system_clock::now();
        const auto seconds = std::chrono::system_clock::to_time_t(now);
        std::tm utc{};
        gmtime_r(&seconds, &utc);
        std::ostringstream name;
        name << safe << '_' << std::put_time(&utc, "%Y%m%dT%H%M%S") << '_'
             << std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count() % 1000000
             << "Z";
        name_ = name.str();
        started_ = Clock::now();
        configured_ = true;
    }

    size_t Input() { return ++inputs_; }

    template<class Track>
    void Measure(System& system, size_t input_index, double timestamp, Track track) {
        const auto begin = Clock::now();
        track();
        const double elapsed = std::chrono::duration<double, std::milli>(Clock::now() - begin).count();
        const auto& frame = system.GetTracker()->mCurrentFrame;
        const int state = system.GetTrackingState();
        const bool valid = frame.isSet() && (state == Tracking::OK || state == Tracking::OK_KLT);
        size_t inliers = 0;
        if (valid) {
            for (size_t i = 0; i < frame.mvpMapPoints.size(); ++i) {
                auto* point = frame.mvpMapPoints[i];
                if (point && i < frame.mvbOutlier.size() && !frame.mvbOutlier[i] &&
                    !point->isBad() && point->Observations() > 0) ++inliers;
            }
        }
        samples_.push_back({input_index, timestamp, elapsed,
            frame.experimentExtractionMs[0], frame.experimentExtractionMs[1], state, valid,
            frame.mvKeys.size(), frame.mvKeysRight.size(), inliers,
            system.GetAtlas()->GetCurrentMap()->GetId()});
    }

    std::filesystem::path Save(System& system) {
        if (!configured_) return {};
        namespace fs = std::filesystem;
        fs::create_directories(root_);
        fs::path directory = fs::path(root_) / name_;
        size_t suffix = 0;
        while (!fs::create_directory(directory)) directory = fs::path(root_) / (name_ + '_' + std::to_string(++suffix));
        for (const auto* sub : {"trajectories", "statistics", "maps", "config"}) fs::create_directory(directory / sub);
        Write(directory / "config/camera.yaml", settings_contents_);
        Write(directory / "config/parameters.txt", parameters_);
        Write(directory / "config/build_revision.txt", "git_revision=" ORBSLAM3_BUILD_REVISION "\nsource_sha256=" ORBSLAM3_SOURCE_SHA256 "\n");
        auto frames = Open(directory / "statistics/frames.csv");
        frames << "processed_index,input_index,timestamp,tracking_state,valid_pose,tracking_ms,extraction_left_ms,extraction_right_ms,features_left,features_right,map_inlier_associations,map_id\n";
        size_t valid = 0, lost_events = 0;
        bool was_lost = false;
        std::vector<double> times;
        for (size_t i = 0; i < samples_.size(); ++i) {
            const auto& s = samples_[i];
            frames << i + 1 << ',' << s.input_index << ',' << s.timestamp << ',' << s.state << ',' << s.valid
                   << ',' << s.tracking_ms << ',' << s.left_ms << ',' << s.right_ms << ',' << s.features_left
                   << ',' << s.features_right << ',' << s.inliers << ',' << s.map_id << '\n';
            valid += s.valid;
            const bool lost = s.state == Tracking::RECENTLY_LOST || s.state == Tracking::LOST;
            if (lost && !was_lost) { ++lost_events; was_lost = true; }
            if (s.valid) was_lost = false;
            times.push_back(s.tracking_ms);
        }
        frames.close();
        auto losses = Open(directory / "statistics/tracking_losses.csv");
        losses << "first_processed_index,start_timestamp,end_timestamp,observed_duration_seconds,recovered\n";
        for (size_t i = 0; i < samples_.size();) {
            const auto lost = [](const Sample& s) {
                return s.state == Tracking::RECENTLY_LOST || s.state == Tracking::LOST;
            };
            if (!lost(samples_[i])) { ++i; continue; }
            const size_t begin = i;
            // Include reinitialization after a loss until a valid pose returns.
            while (i < samples_.size() && !samples_[i].valid) ++i;
            const double end = samples_[i < samples_.size() ? i : i - 1].timestamp;
            losses << begin + 1 << ',' << samples_[begin].timestamp << ',' << end << ','
                   << std::max(0.0, end - samples_[begin].timestamp) << ','
                   << (i < samples_.size() && samples_[i].valid) << '\n';
        }
        losses.close();
        size_t keyframes = 0, points = 0, exported_maps = 0;
        auto index = Open(directory / "maps/index.csv");
        index << "map_id,keyframes,points,origin_keyframe_id\n";
        for (auto* map : system.GetAtlas()->GetAllMaps()) {
            auto kfs = map->GetAllKeyFrames();
            kfs.erase(std::remove_if(kfs.begin(), kfs.end(), [](KeyFrame* k) { return !k || k->isBad(); }), kfs.end());
            if (kfs.empty()) continue;
            ++exported_maps;
            std::sort(kfs.begin(), kfs.end(), KeyFrame::lId);
            // Each independent map has its own first-keyframe origin; never merge unrelated maps.
            const auto origin_inverse = kfs.front()->GetPose();
            const auto stem = "map_" + std::to_string(map->GetId());
            auto trajectory = Open(directory / "trajectories" / (stem + "_keyframes_tum.txt"));
            for (auto* kf : kfs) {
                const auto pose = origin_inverse * kf->GetPoseInverse();
                const auto t = pose.translation();
                const auto q = pose.unit_quaternion();
                trajectory << kf->mTimeStamp << ' ' << t.x() << ' ' << t.y() << ' ' << t.z()
                           << ' ' << q.x() << ' ' << q.y() << ' ' << q.z() << ' ' << q.w() << '\n';
            }
            trajectory.close();
            // Frame trajectories reconstructed from final reference-keyframe poses, per map.
            auto camera = Open(directory / "trajectories" / (stem + "_camera_tum.txt"));
            auto camera_kitti = Open(directory / "trajectories" / (stem + "_camera_kitti.txt"));
            auto* tracker = system.GetTracker();
            auto reference = tracker->mlpReferences.begin();
            auto timestamp = tracker->mlFrameTimes.begin();
            auto lost = tracker->mlbLost.begin();
            for (const auto& relative : tracker->mlRelativeFramePoses) {
                if (reference == tracker->mlpReferences.end() || timestamp == tracker->mlFrameTimes.end() ||
                    lost == tracker->mlbLost.end()) break;
                auto* kf = *reference++;
                const double time = *timestamp++;
                const bool missing = *lost++;
                if (missing || !kf) continue;
                Sophus::SE3f correction;
                while (kf && kf->isBad()) {
                    correction = correction * kf->mTcp;
                    kf = kf->GetParent();
                }
                if (!kf || kf->GetMap() != map) continue;
                const auto pose = origin_inverse * (relative * correction * kf->GetPose()).inverse();
                const auto t = pose.translation();
                const auto q = pose.unit_quaternion();
                camera << time << ' ' << t.x() << ' ' << t.y() << ' ' << t.z()
                       << ' ' << q.x() << ' ' << q.y() << ' ' << q.z() << ' ' << q.w() << '\n';
                const auto rotation = pose.rotationMatrix();
                for (int r = 0; r < 3; ++r) {
                    for (int c = 0; c < 3; ++c) camera_kitti << rotation(r, c) << ' ';
                    camera_kitti << t(r) << (r == 2 ? '\n' : ' ');
                }
            }
            camera.close();
            camera_kitti.close();
            std::vector<Eigen::Vector3f> positions;
            for (auto* point : map->GetAllMapPoints()) {
                if (!point || point->isBad()) continue;
                const Eigen::Vector3f p = origin_inverse * point->GetWorldPos();
                if (p.allFinite()) positions.push_back(p);
            }
            auto cloud = Open(directory / "maps" / (stem + "_sparse.ply"));
            cloud << "ply\nformat ascii 1.0\ncomment coordinates relative to first valid keyframe\nelement vertex "
                  << positions.size() << "\nproperty float x\nproperty float y\nproperty float z\nend_header\n";
            for (const auto& p : positions) cloud << p.x() << ' ' << p.y() << ' ' << p.z() << '\n';
            cloud.close();
            index << map->GetId() << ',' << kfs.size() << ',' << positions.size() << ',' << kfs.front()->mnId << '\n';
            keyframes += kfs.size();
            points += positions.size();
        }
        index.close();
        // Legacy names are only unambiguous when there is one nonempty map.
        if (exported_maps == 1 && !system.GetAtlas()->GetAllKeyFrames().empty()) {
            const auto tum = directory / "trajectories/camera_tum.txt";
            const auto kitti = directory / "trajectories/camera_kitti.txt";
            system.SaveTrajectoryTUM(tum.string());
            system.SaveTrajectoryKITTI(kitti.string());
            if (!fs::exists(tum) || !fs::exists(kitti)) throw std::runtime_error("Trajectory export failed");
        }
        double sum = 0;
        for (double t : times) sum += t;
        std::sort(times.begin(), times.end());
        auto percentile = [&](double p) { return times.empty() ? 0.0 : times[static_cast<size_t>(std::ceil(p * times.size())) - 1]; };
        auto summary = Open(directory / "statistics/summary.txt");
        summary << "sensor_mode=" << mode_ << "\nsettings_source=" << settings_path_
                << "\nreceived_images_or_synchronized_pairs=" << inputs_
                << "\ncompleted_tracking_calls=" << samples_.size()
                << "\nunprocessed_or_failed_inputs=" << inputs_ - samples_.size()
                << "\nvalid_pose_frames=" << valid
                << "\nvalid_pose_ratio_processed=" << (samples_.empty() ? 0.0 : double(valid) / samples_.size())
                << "\nvalid_pose_ratio_received=" << (inputs_ ? double(valid) / inputs_ : 0.0)
                << "\ntracking_loss_events=" << lost_events
                << "\ntracking_ms_mean=" << (times.empty() ? 0.0 : sum / times.size())
                << "\ntracking_ms_median=" << (times.empty() ? 0.0 : (times[(times.size()-1)/2] + times[times.size()/2]) / 2)
                << "\ntracking_ms_p95=" << percentile(0.95)
                << "\ntracking_ms_max=" << (times.empty() ? 0.0 : times.back())
                << "\nexported_maps=" << exported_maps
                << "\nexported_keyframes=" << keyframes << "\nexported_sparse_points=" << points
                << "\nwall_seconds_through_export=" << std::chrono::duration<double>(Clock::now()-started_).count() << '\n';
        summary.close();
        Write(directory / "README.txt",
            "Input index counts monocular callbacks or synchronized image pairs, not dataset frame IDs.\n"
            "DDS drops and pairs rejected by the synchronizer are not observable by this counter.\n"
            "frames.csv contains completed tracking calls, including initialization and lost states.\n"
            "State: -1=not ready, 0=no images, 1=initializing, 2=OK, 3=recently lost, 4=lost, 5=OK_KLT.\n"
            "valid_pose requires an assigned pose and state 2 or 5; predicted lost poses are excluded.\n"
            "Loss events start at state 3/4 and include reinitialization until a valid pose returns.\n"
            "Initial startup is excluded. An unrecovered interval ends at the last observed sample.\n"
            "recovered=0 marks an interval truncated by shutdown, not a confirmed recovery.\n"
            "tracking_ms measures the synchronous Track call, excluding ROS conversion and recorder work.\n"
            "It is not total CPU cost including asynchronous mapping/loop closing.\n"
            "Extraction durations are per-camera ORB feature extraction calls, including descriptors.\n"
            "Stereo cameras run in parallel; do not sum their durations as wall time.\n"
            "map_inlier_associations counts non-outlier persistent map-point associations when valid.\n"
            "It is not the number of unique map points or the optimizer's internal inlier counter.\n"
            "Independent maps are exported separately, each relative to its first valid keyframe.\n"
            "Camera/keyframe TUM trajectories and sparse PLY files with the same map ID share coordinates.\n"
            "camera_tum/kitti aliases are present only for a single nonempty current map.\n"
            "Use per-map trajectories for multi-map runs; no unaligned merged trajectory is written.\n"
            "Online valid-pose counts may exceed final trajectory rows if maps were reset/discarded.\n"
            "Monocular coordinates have unknown scale. Sparse maps are not dense surface reconstructions.\n"
            "No ground truth or accuracy metric is inferred. Configuration is captured at initialization.\n"
            "Save runs on normal SLAM shutdown, not automatically at dataset-player EOF.\n");
        return directory;
    }
private:
    static std::ofstream Open(const std::filesystem::path& path) {
        std::ofstream out;
        out.exceptions(std::ios::badbit | std::ios::failbit);
        out.open(path);
        out << std::setprecision(17);
        return out;
    }
    static void Write(const std::filesystem::path& path, const std::string& content) {
        auto out = Open(path);
        out << content;
        out.close();
    }
    bool configured_ = false;
    size_t inputs_ = 0;
    std::string root_, name_, mode_, settings_path_, settings_contents_, parameters_;
    Clock::time_point started_;
    std::vector<Sample> samples_;
};
} // namespace ORB_SLAM3
#endif
