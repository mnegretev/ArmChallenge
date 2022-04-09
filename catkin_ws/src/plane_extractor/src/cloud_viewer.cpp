#include <opencv2/opencv.hpp>
#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Image.h"
#include "tf/transform_listener.h"
#include "pcl_ros/transforms.h"

bool save_cloud = false;
tf::TransformListener* tf_listener;

void PointCloud2MsgToCvMat(const sensor_msgs::PointCloud2::ConstPtr& pc_msg, cv::Mat& bgr_dest, cv::Mat& pc_dest)
{
    bgr_dest = cv::Mat::zeros(pc_msg->height, pc_msg->width, CV_8UC3);
    pc_dest  = cv::Mat::zeros(pc_msg->height, pc_msg->width, CV_32FC3);
    for(int i=0; i < bgr_dest.cols; i++)
        for(int j=0; j < bgr_dest.rows; j++)
        {
            float* x = (float*)&pc_msg->data[(j*pc_msg->width + i)*pc_msg->point_step    ];
            float* y = (float*)&pc_msg->data[(j*pc_msg->width + i)*pc_msg->point_step + 4];
            float* z = (float*)&pc_msg->data[(j*pc_msg->width + i)*pc_msg->point_step + 8];
            pc_dest.at<cv::Vec3f>(j, i)[0] = *x;
            pc_dest.at<cv::Vec3f>(j, i)[1] = *y;
            pc_dest.at<cv::Vec3f>(j, i)[2] = *z;
            bgr_dest.data[j*bgr_dest.step + i*3]     = pc_msg->data[(j*pc_msg->width + i)*pc_msg->point_step + 18];
            bgr_dest.data[j*bgr_dest.step + i*3 + 1] = pc_msg->data[(j*pc_msg->width + i)*pc_msg->point_step + 17];
            bgr_dest.data[j*bgr_dest.step + i*3 + 2] = pc_msg->data[(j*pc_msg->width + i)*pc_msg->point_step + 16];
        }
}


void callback_point_cloud(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    cv::Mat bgr;
    cv::Mat xyz;
    PointCloud2MsgToCvMat(msg, bgr, xyz);
    cv::imshow("BGR", bgr);
    char cmd = cv::waitKey(10);
    if(cmd == 's')
    {
        std::cout << "Saving BGR image..." << std::endl;
        cmd = 0;
        cv::imwrite("BGR.jpg", bgr);
        std::cout << "Transforming point cloud to world frame..." << std::endl;
        sensor_msgs::PointCloud2 temp = *msg;
        pcl_ros::transformPointCloud("world", temp, temp, *tf_listener);
        PointCloud2MsgToCvMat(msg, bgr, xyz);
        cv::FileStorage fs("XYZ.yaml",  cv::FileStorage::WRITE);
        fs << "xyz" << xyz;
    }
}

void callback_save_cloud(const std_msgs::Empty::ConstPtr& msg)
{
    save_cloud = true;
}

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING CLOUD VIEWER BY MARCOSOFT ..." << std::endl;
    ros::init(argc, argv, "cloud_capturer");
    ros::NodeHandle n;
    ros::Subscriber sub_cloud = n.subscribe("/camera/depth/points", 1, callback_point_cloud);
    ros::Subscriber sub_save  = n.subscribe("/save_cloud", 1, callback_save_cloud);
    ros::Rate loop(30);
    tf_listener = new tf::TransformListener();
    while(ros::ok())
    {
        ros::spinOnce();
        loop.sleep();
    }
    return 0;
}
