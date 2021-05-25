#pragma once

#include <map>
#include <cv_bridge/cv_bridge.h>
#include "feature_point_3d.hpp"
#include "orb_features_data.hpp"

#include <sophus/se3.hpp>
namespace sara_slam{
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
}