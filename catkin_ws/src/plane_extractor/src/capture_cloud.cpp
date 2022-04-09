#include <opencv2/opencv.hpp>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Image.h"
#include "tf/transform_listener.h"
#include "pcl_ros/transforms.h"

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING CLOUD CAPTURER BY MARCOSOFT ..." << std::endl;
    ros::init(argc, argv, "cloud_capturer");
    ros::NodeHandle n;
    tf::TransformListener tf_listener;
    sensor_msgs::PointCloud2 cloud = *ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/camera/depth/points");
    pcl_ros::transformPointCloud("world", cloud, cloud, tf_listener);
}
