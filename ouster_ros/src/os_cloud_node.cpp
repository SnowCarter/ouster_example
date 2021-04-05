/**
 * @file
 * @brief Example node to publish point clouds and imu topics
 */

#include <ros/console.h>
#include <ros/ros.h>
#include <ros/service.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <algorithm>
#include <chrono>
#include <memory>

#include "ouster/lidar_scan.h"
#include "ouster/types.h"
#include "ouster_ros/OSConfigSrv.h"
#include "ouster_ros/PacketMsg.h"
#include "ouster_ros/ros.h"

using PacketMsg = ouster_ros::PacketMsg;
using Cloud = ouster_ros::Cloud;
using Point = ouster_ros::Point;
namespace sensor = ouster::sensor;

// template <class ContainerAllocator>

typedef std::vector<uint8_t, std::allocator<void>::rebind<uint8_t>::other>::const_iterator pm_buf_iter;
typedef std::vector<uint8_t, std::allocator<void>::rebind<uint8_t>::other> pm_buf_type;

int main(int argc, char** argv) {
    ros::init(argc, argv, "os_cloud_node");
    ros::NodeHandle nh("~");

    auto tf_prefix = nh.param("tf_prefix", std::string{});
    if (!tf_prefix.empty() && tf_prefix.back() != '/') tf_prefix.append("/");
    auto sensor_frame = tf_prefix + "os_sensor";
    auto imu_frame = tf_prefix + "os_imu";
    auto lidar_frame = tf_prefix + "os_lidar";

    ouster_ros::OSConfigSrv cfg{};
    auto client = nh.serviceClient<ouster_ros::OSConfigSrv>("os_config");
    client.waitForExistence();
    if (!client.call(cfg)) {
        ROS_ERROR("Calling config service failed");
        return EXIT_FAILURE;
    }

    auto info = sensor::parse_metadata(cfg.response.metadata);
    uint32_t H = info.format.pixels_per_column;
    uint32_t W = info.format.columns_per_frame;

    uint32_t W_sub = info.format.columns_per_frame/12; // 360 degree divided by 30 degree is 12 parts. 
    uint32_t H_sub = info.format.pixels_per_column; 

    auto pf = sensor::get_format(info);

    auto lidar_pub = nh.advertise<sensor_msgs::PointCloud2>("points", 10);
    auto subCloud_pub = nh.advertise<sensor_msgs::PointCloud2>("sub_points", 10);
    auto imu_pub = nh.advertise<sensor_msgs::Imu>("imu", 100);

    auto xyz_lut = ouster::make_xyz_lut(info);

    Cloud cloud{W, H};
    ouster::LidarScan ls{W, H};

    Cloud cloud_sub{W_sub, H_sub}; 
    ouster::LidarScan ls_sub{W_sub, H_sub}; 

    ouster::ScanBatcher batch(W, pf);
    ouster::ScanBatcher batch_sub(W_sub, pf);

    auto lidar_handler = [&](const PacketMsg& pm) mutable {
        //for whole cloud
        if (batch(pm.buf.data(), ls)) {//TODO: packetmsg to lidar scan
            auto h = std::find_if(
                ls.headers.begin(), ls.headers.end(), [](const auto& h) {
                    return h.timestamp != std::chrono::nanoseconds{0};
                });
            if (h != ls.headers.end()) {
                scan_to_cloud(xyz_lut, h->timestamp, ls, cloud);
                lidar_pub.publish(ouster_ros::cloud_to_cloud_msg(
                    cloud, h->timestamp, sensor_frame));
            }
        }
        //for sub cloud
        
        pm_buf_iter first_sub = pm.buf.begin(); 
        pm_buf_iter last_sub;
        bool isLastCloud = false; 
        while(!isLastCloud){
            // cut the package into pieces 
            last_sub = first_sub + W_sub;
            if(last_sub > pm.buf.end()) {//TODO: check if it better of worse(for the last piece of msg, if the size is less than W_sub, use previeous msg to replace it)
                last_sub = pm.buf.end(); 
                first_sub = last_sub - W_sub; 
                isLastCloud = true; 
            }
            pm_buf_type pm_buf_sub(first_sub, last_sub); // create a new package message with a piece of data. about 30 degree. 

            // convert the package msg into laser scan
            if (batch_sub(pm_buf_sub.data(), ls_sub)) {
                auto h = std::find_if(
                    ls_sub.headers.begin(), ls_sub.headers.end(), [](const auto& h) {
                        return h.timestamp != std::chrono::nanoseconds{0};
                    });
                if (h != ls_sub.headers.end()) {
                    scan_to_cloud(xyz_lut, h->timestamp, ls_sub, cloud_sub);
                    subCloud_pub.publish(ouster_ros::cloud_to_cloud_msg(
                        cloud_sub, h->timestamp, sensor_frame));
                }
            }
            first_sub = first_sub + W_sub + 1; 

        }
        
    };

    auto imu_handler = [&](const PacketMsg& p) {
        imu_pub.publish(ouster_ros::packet_to_imu_msg(p, imu_frame, pf));
    };

    auto lidar_packet_sub = nh.subscribe<PacketMsg, const PacketMsg&>(
        "lidar_packets", 2048, lidar_handler);
    auto imu_packet_sub = nh.subscribe<PacketMsg, const PacketMsg&>(
        "imu_packets", 100, imu_handler);

    // publish transforms
    tf2_ros::StaticTransformBroadcaster tf_bcast{};

    tf_bcast.sendTransform(ouster_ros::transform_to_tf_msg(
        info.imu_to_sensor_transform, sensor_frame, imu_frame));

    tf_bcast.sendTransform(ouster_ros::transform_to_tf_msg(
        info.lidar_to_sensor_transform, sensor_frame, lidar_frame));

    ros::spin();

    return EXIT_SUCCESS;
}
