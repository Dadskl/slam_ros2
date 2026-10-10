#include "AgastCornerDetector.h"

#include <opencv2/features2d.hpp>

namespace ORB_SLAM3
{

AgastCornerDetector::AgastCornerDetector(
    int initialThreshold,
    int minimumThreshold)
    : initialThreshold_(initialThreshold),
      minimumThreshold_(minimumThreshold)
{
}

void AgastCornerDetector::Detect(
    const cv::Mat& image,
    std::vector<cv::KeyPoint>& keypoints,
    bool relaxed) const
{
    keypoints.clear();

    const int threshold =
        relaxed ? minimumThreshold_ : initialThreshold_;

    cv::AGAST(
        image,
        keypoints,
        threshold,
        true,
        cv::AgastFeatureDetector::OAST_9_16);
}

}  // namespace ORB_SLAM3