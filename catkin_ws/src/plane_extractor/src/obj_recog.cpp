#include <opencv2/opencv.hpp>
#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Image.h"
#include "visualization_msgs/Marker.h"
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

cv::Mat get_image_no_background(cv::Mat bgr, cv::Mat xyz_camera, cv::Mat xyz_robot)
{
    for(int i=0; i < bgr.rows; i++)
        for(int j=0; j < bgr.cols; j++)
        {
            cv::Vec3f pr = xyz_robot.at <cv::Vec3f>(i,j);
            cv::Vec3f pc = xyz_camera.at<cv::Vec3f>(i,j);
            if(pr[2] < (z_plane_threshold/1000.0) || pc[2] < (dist_threshold/100.0) || isnan(pc[0]))
            {
                bgr.at<cv::Vec3b>(i,j) = cv::Vec3b(0,0,0);
            }
        }
    return bgr;
}

cv::Mat color_segmentation(cv::Mat bgr)
{
    cv::Mat hsv;
    cv::cvtColor(bgr, hsv, CV_BGR2HSV);
    cv::Mat bin_b, bin_g, bin_r, bin_y, bin_objects;
    cv::inRange(hsv, cv::Scalar(110, 200, 30), cv::Scalar(130, 255, 255), bin_b);
    cv::inRange(hsv, cv::Scalar( 50, 200, 30), cv::Scalar( 70, 255, 255), bin_g);
    cv::inRange(hsv, cv::Scalar(  0, 200, 30), cv::Scalar( 10, 255, 255), bin_r);
    cv::inRange(hsv, cv::Scalar( 20, 200, 30), cv::Scalar( 40, 255, 255), bin_y);
    cv::bitwise_or(bin_b, bin_g, bin_objects);
    cv::bitwise_or(bin_objects, bin_r, bin_objects);
    cv::bitwise_or(bin_objects, bin_y, bin_objects);
    cv::imshow("Objects", bin_objects);
    return bin_objects;
}

cv::Mat clusterize(cv::Mat bin, cv::Mat xyz_robot)
{
    std::vector<cv::Vec3f> filtered;
    cv::Mat labels, centers;
    for(int i=0; i < bin.rows; i++)
        for(int j=0; j < bin.cols; j++)
            if(bin.at<unsigned char>(i,j) != 0)
                filtered.push_back(xyz_robot.at<cv::Vec3f>(i,j));
    cv::kmeans(filtered, 5, labels, cv::TermCriteria( cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 10, 1.0),
               3, cv::KMEANS_PP_CENTERS, centers);
    //std::cout << "Found " << centers.rows<< " centroids from  " << filtered.size() << " points" << std::endl;
    return centers;
}

visualization_msgs::Marker get_object_marker(cv::Mat centroids)
{
    visualization_msgs::Marker mrk;
    
}

void callback_point_cloud(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    sensor_msgs::PointCloud2 cloud_camera = *msg;
    sensor_msgs::PointCloud2 cloud_robot;
    pcl_ros::transformPointCloud("world", cloud_camera, cloud_robot, *tf_listener);
    cv::Mat bgr, bin, centroids;
    cv::Mat xyz_robot;
    cv::Mat xyz_camera;
    PointCloud2MsgToCvMat(cloud_camera, bgr, xyz_camera);
    PointCloud2MsgToCvMat(cloud_robot , bgr, xyz_robot );
    if(bgr.cols < 1 || bgr.rows < 1)
    {
        std::cout << "Invalid point cloud ..." << std::endl;
        return;
    }
    bgr = get_image_no_background(bgr, xyz_camera, xyz_robot);
    bin = color_segmentation(bgr);
    centroids = clusterize(bin, xyz_robot);
    cv::imshow("Extracted Plane", bgr);
}

void on_z_changed(int, void*){}
void on_d_changed(int, void*){}

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING OBJECT RECOGNIZER BY MARCOSOFT ..." << std::endl;
    ros::init(argc, argv, "obj_recog");
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
