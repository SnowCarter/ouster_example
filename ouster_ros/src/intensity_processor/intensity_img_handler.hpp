#include <ros/ros.h>
#include <ros/package.h>

#include <sensor_msgs/Image.h>
// #include <DBoW2/DBoW2.h>
#include <DBoW3/DBoW3.h>

#include <opencv2/features2d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/eigen.hpp>
#include <opencv/cv.h>

#include "keyframe_data.hpp"
// #include "ouster_ros/ros.h"

class intensityImgHandler
{
private:
    /* data */
    
public:
    intensityImgHandler(/* args */);
    ~intensityImgHandler();
    void intensityImgProcessor(const sensor_msgs::PointCloud2::ConstPtr& cloudMsg, const sensor_msgs::Image &image_intensity); 
    int detectLoop(cv::Mat & image, cv::Mat& descriptors, int frame_index);
    sara_slam::OrbFeaturesData extractOrbFeatures(double imageTime, const cv::Mat & image);
    sensor_msgs::ImagePtr cvMat2Image(std_msgs::Header header, cv::Mat & image);

public:   
    DBoW3::Database db;
    DBoW3::Vocabulary* voc;
    std::vector<cv::Mat> bow_descriptors;
    std::map<int, cv::Mat> image_pool;
    // cv::Mat MASK;
};

intensityImgHandler::intensityImgHandler(/* args */)
{
    std::string PROJECT_NAME("ouster_ros");
    std::string pkg_path = ros::package::getPath(PROJECT_NAME);
    std::string vocabulary_file("/src/intensity_processor/orbvoc.dbow3");
    vocabulary_file = pkg_path + vocabulary_file;
    voc= new DBoW3::Vocabulary(vocabulary_file); 
    db.setVocabulary(*voc, false, 0);
}

intensityImgHandler::~intensityImgHandler()
{
}

sensor_msgs::ImagePtr intensityImgHandler::cvMat2Image(std_msgs::Header header, cv::Mat & image){
    static cv_bridge::CvImage outImg; 
    outImg.header = header;
    outImg.encoding = "bgr8";
    outImg.image = image;
    auto imageOut = outImg.toImageMsg();
    return imageOut; 
}

void intensityImgHandler::intensityImgProcessor(const sensor_msgs::PointCloud2::ConstPtr &cloudMsg, const sensor_msgs::Image &image_intensity){
    //get key frame
    // MASK = cv::Mat(image_intensity.height, image_intensity.width, CV_8UC1, cv::Scalar(255)); 
    static int global_frame_index = 0;
    double image_time = cloudMsg->header.stamp.toSec(); 
    static double last_skip_time = -1; 
    if(image_time - last_skip_time < 0.15) return;
    else last_skip_time = image_time; 

    
    // sara_slam::KeyframeData keyframe;

    cv_bridge::CvImagePtr cv_ptr; 
    cv_ptr = cv_bridge::toCvCopy(image_intensity);
    // keyframe.orbFeatureData = extractOrbFeatures(image_time, cv_ptr->image);

    cv::Mat descriptors;
    std::vector<cv::KeyPoint> keypoints;
    cv::Ptr<cv::ORB> detector = cv::ORB::create(2500, 1.2f, 8, 1);
    detector->detectAndCompute(cv_ptr->image, cv::Mat(), keypoints, descriptors);
    // bow_descriptors.resize(descriptors.rows);

    //show features
    cv::Mat imgShow;

    cv::drawKeypoints(cv_ptr->image, keypoints, imgShow, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT );
    cv::resize(imgShow, imgShow, cv::Size(), 2, 2);
    cv::imshow("BOW Keypoints", imgShow );
    cv::waitKey(10);
    // detect loop
    int loop_intex = detectLoop(cv_ptr->image, descriptors, global_frame_index);
    if(loop_intex > 1){
        cv::Mat imgShowTmp; 
        imgShowTmp = cv_ptr->image.clone(); 
        putText(imgShowTmp, "Index: " + std::to_string(global_frame_index), cv::Point2f(10, 50), CV_FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255), 2);

        auto it = image_pool.find(loop_intex);
        cv::Mat tmp_image = (it->second).clone();
        putText(tmp_image, "Index:  " + std::to_string(loop_intex) + ", loop candidate" , cv::Point2f(10, 50), CV_FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255), 2);
        cv::vconcat(imgShowTmp, tmp_image, imgShowTmp);
        cv::imshow("loop candidate", imgShowTmp);
        cv::waitKey(10);
    }




    global_frame_index++;
    

    
}

int intensityImgHandler::detectLoop(cv::Mat & image, cv::Mat & descriptors, int frame_index){
    DBoW3::QueryResults ret;
    int MIN_LOOP_SEARCH_GAP = 50;
    float MIN_LOOP_BOW_TH = 0.0015;
    db.query(descriptors, ret, 4, frame_index - MIN_LOOP_SEARCH_GAP);
    db.add(descriptors);

    image_pool[frame_index] = image.clone();
    cv::Mat bow_images = image.clone();
    // if (ret.size() > 0)
    //         putText(bow_images, "Index: " + std::to_string(frame_index), cv::Point2f(10, 50), CV_FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255), 2);
    // for (unsigned int i = 0; i < ret.size(); i++)
    // {
    //     int tmp_index = ret[i].Id;
    //     auto it = image_pool.find(tmp_index);
    //     cv::Mat tmp_image = (it->second).clone();
    //     putText(tmp_image, "Index:  " + std::to_string(tmp_index) + ", BoW score:" + std::to_string(ret[i].Score), cv::Point2f(10, 50), CV_FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255), 2);
    //     cv::vconcat(bow_images, tmp_image, bow_images);
    // }

    // cv::imshow("BoW images", bow_images);
    // cv::waitKey(10);

    if (frame_index - MIN_LOOP_SEARCH_GAP < 0)
        return -1;

    bool find_loop = false;
    if (ret.size() >= 1 && ret[0].Score > MIN_LOOP_BOW_TH)
    {
        for (unsigned int i = 1; i < ret.size(); i++)
        {
            if (ret[i].Score > MIN_LOOP_BOW_TH)
            {          
                find_loop = true;
            }
        }
    }
    
    if (find_loop && frame_index > 5)
    {
        int min_index = -1;
        for (unsigned int i = 0; i < ret.size(); i++)
        {
            if (min_index == -1 || ((int)ret[i].Id < min_index && ret[i].Score > 0.015))
                min_index = ret[i].Id;
        }
        return min_index;
    }
    else
        return -1;

}
sara_slam::OrbFeaturesData intensityImgHandler::extractOrbFeatures(double imageTime, const cv::Mat & image){
    cv::Mat mask;
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;

    cv::Ptr<cv::ORB> orb = cv::ORB::create();
    orb->detectAndCompute(image, mask, keypoints, descriptors);
    std::vector<cv::Mat> features;
    features.resize(static_cast<size_t>(descriptors.rows));

    for (int i = 0; i < descriptors.rows; ++i)
    {
        features[i] = descriptors.row(i);
    }
    sara_slam::OrbFeaturesData orbfeature{imageTime, keypoints, features, descriptors};



    return orbfeature;
}