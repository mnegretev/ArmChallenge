#include <opencv2/opencv.hpp>
#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Image.h"
#include "tf/transform_listener.h"
#include "pcl_ros/transforms.h"

tf::TransformListener* tf_listener;
int z_plane_threshold = 10;
int dist_threshold    = 10;

void PointCloud2MsgToCvMat(sensor_msgs::PointCloud2& pc_msg, cv::Mat& bgr_dest, cv::Mat& pc_dest)
{
    bgr_dest = cv::Mat::zeros(pc_msg.height, pc_msg.width, CV_8UC3);
    pc_dest  = cv::Mat::zeros(pc_msg.height, pc_msg.width, CV_32FC3);
    for(int i=0; i < bgr_dest.cols; i++)
        for(int j=0; j < bgr_dest.rows; j++)
        {
            float* x = (float*)&pc_msg.data[(j*pc_msg.width + i)*pc_msg.point_step    ];
            float* y = (float*)&pc_msg.data[(j*pc_msg.width + i)*pc_msg.point_step + 4];
            float* z = (float*)&pc_msg.data[(j*pc_msg.width + i)*pc_msg.point_step + 8];
            pc_dest.at<cv::Vec3f>(j, i)[0] = *x;
            pc_dest.at<cv::Vec3f>(j, i)[1] = *y;
            pc_dest.at<cv::Vec3f>(j, i)[2] = *z;
            bgr_dest.data[j*bgr_dest.step + i*3]     = pc_msg.data[(j*pc_msg.width + i)*pc_msg.point_step + 18];
            bgr_dest.data[j*bgr_dest.step + i*3 + 1] = pc_msg.data[(j*pc_msg.width + i)*pc_msg.point_step + 17];
            bgr_dest.data[j*bgr_dest.step + i*3 + 2] = pc_msg.data[(j*pc_msg.width + i)*pc_msg.point_step + 16];
        }
}


void callback_point_cloud(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    sensor_msgs::PointCloud2 cloud_camera = *msg;
    sensor_msgs::PointCloud2 cloud_robot;
    pcl_ros::transformPointCloud("world", cloud_camera, cloud_robot, *tf_listener);
    cv::Mat bgr;
    cv::Mat xyz_robot;
    cv::Mat xyz_camera;
    PointCloud2MsgToCvMat(cloud_camera, bgr, xyz_camera);
    PointCloud2MsgToCvMat(cloud_robot , bgr, xyz_robot );
    if(bgr.cols < 1 || bgr.rows < 1)
    {
        std::cout << "Invalid point cloud ..." << std::endl;
        return;
    }
    for(int i=0; i < bgr.rows; i++)
        for(int j=0; j < bgr.cols; j++)
        {
            cv::Vec3f pr = xyz_robot.at <cv::Vec3f>(i,j);
            cv::Vec3f pc = xyz_camera.at<cv::Vec3f>(i,j);
            if(fabs(pr[2]) < (z_plane_threshold/1000.0) || pc[2] < (dist_threshold/100.0) || isnan(pc[0]))
            {
                bgr.at<cv::Vec3b>(i,j) = cv::Vec3b(0,0,0);
            }
        }
    cv::imshow("Extracted Plane", bgr);
}

void on_z_changed(int, void*){}
void on_d_changed(int, void*){}

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING PLANE EXTRACTOR BY MARCOSOFT ..." << std::endl;
    ros::init(argc, argv, "cloud_capturer");
    ros::NodeHandle n;
    ros::Subscriber sub_cloud = n.subscribe("/camera/depth/points", 1, callback_point_cloud);
    ros::Rate loop(30);
    tf_listener = new tf::TransformListener();

    cv::namedWindow("Extracted Plane");
    cv::createTrackbar("Z:", "Extracted Plane", &z_plane_threshold,  50, on_z_changed);
    cv::createTrackbar("D:", "Extracted Plane", &dist_threshold   , 100, on_d_changed);
    
    while(ros::ok() && cv::waitKey(10) != 27)
    {
        ros::spinOnce();
        loop.sleep();
    }
    return 0;
}
