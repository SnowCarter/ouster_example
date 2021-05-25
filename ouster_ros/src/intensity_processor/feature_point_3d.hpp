#pragma once

#include <opencv2/core.hpp>

namespace sara_slam{
struct FeaturePoint3d
{
    size_t featureId;
    cv::Point2d pointInPixel;
    cv::Point3d pointInCamera;
    cv::Point3d point3d;
};
}

