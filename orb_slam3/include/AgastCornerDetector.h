#ifndef ORB_SLAM3_AGAST_CORNER_DETECTOR_H
#define ORB_SLAM3_AGAST_CORNER_DETECTOR_H

#include "ORBextractor.h"

namespace ORB_SLAM3
{

class AgastCornerDetector final : public CornerDetector
{
public:
    AgastCornerDetector(int initialThreshold, int minimumThreshold);

    void Detect(
        const cv::Mat& image,
        std::vector<cv::KeyPoint>& keypoints,
        bool relaxed = false) const override;

private:
    int initialThreshold_;
    int minimumThreshold_;
};

}  // namespace ORB_SLAM3

#endif