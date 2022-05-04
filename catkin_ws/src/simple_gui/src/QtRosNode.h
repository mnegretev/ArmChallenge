#pragma once
#include <iostream>
#include <cmath>
#include <QThread>
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include "sensor_msgs/JointState.h"
#include "control_msgs/GripperCommandActionGoal.h"
#include "manip_msgs/InverseKinematicsForPose.h"
#include "manip_msgs/ForwardKinematics.h"

class QtRosNode : public QThread
{
Q_OBJECT
public:
    QtRosNode();
    ~QtRosNode();

    ros::NodeHandle* n;
    ros::Publisher pubGoalQ;
    ros::Publisher pubGoalGrip;
    ros::Publisher pubJointStates;
    ros::Subscriber subJointStates;
    ros::ServiceClient cltInverseKinematics;
    ros::ServiceClient cltForwardKinematics;
    
    bool gui_closed;
    std::vector<float> current_q;
    std::vector<float> current_cartesian;
    
    void run();
    void setNodeHandle(ros::NodeHandle* nh);

    void publish_q_goal_angles(float q1, float q2, float q3, float q4, float q5, float q6, float q7);
    void publish_goal_gripper(float a);
    bool call_inverse_kinematics(std::vector<float>& cartesian, std::vector<float>& articular);
    bool call_forward_kinematics(std::vector<float>& articular, std::vector<float>& cartesian);
    void callback_joint_states(const sensor_msgs::JointState::ConstPtr& msg);
    
signals:
    void updateGraphics();
    void onRosNodeFinished();
    
};
