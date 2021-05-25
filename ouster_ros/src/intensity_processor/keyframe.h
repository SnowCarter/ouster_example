#pragma once
#include <opencv2/core/core.hpp>
#include <vector>
#include <map>
#include <cv_bridge/cv_bridge.h>
#include <sophus/se3.hpp>




struct FeaturePoint3d
{
    size_t featureId;
    cv::Point2d pointInPixel;
    cv::Point3d pointInCamera;
    cv::Point3d point3d;
};


struct OrbFeaturesData
{
    double time;
    std::vector<cv::KeyPoint> keypoints;
    std::vector<cv::Mat> features;
    cv::Mat descriptors;
};

struct KeyframeData
{
    double time;
    cv_bridge::CvImage image;
    Sophus::SE3d pose;
    std::map<size_t, FeaturePoint3d> featurePoints;
    OrbFeaturesData orbFeatureData;
    OrbFeaturesData orbFeaturesDataFromCorners;
    std::map<size_t, size_t> descriptorIdToFeatureId;
};