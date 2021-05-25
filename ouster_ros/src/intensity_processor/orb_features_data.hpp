#pragma once

#include <opencv2/core/core.hpp>
#include <vector>
namespace sara_slam{

struct OrbFeaturesData
{
    double time;
    std::vector<cv::KeyPoint> keypoints;
    std::vector<cv::Mat> features;
    cv::Mat descriptors;
};
    
}
