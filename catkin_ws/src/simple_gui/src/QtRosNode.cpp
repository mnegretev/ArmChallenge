#include "QtRosNode.h"

QtRosNode::QtRosNode()
{
    this->gui_closed = false;
    this->current_q.resize(7);
}

QtRosNode::~QtRosNode()
{
}

void QtRosNode::run()
{    
    ros::Rate loop(10);
    subJointStates = n->subscribe("/my_gen3/joint_states", 10, &QtRosNode::callback_joint_states, this);
    pubGoalQ       = n->advertise<std_msgs::Float32MultiArray>("/goal_pose", 10);
    cltForwardKinematics = n->serviceClient<manip_msgs::ForwardKinematics>("/manipulation/forward_kinematics");
    while(ros::ok() && !this->gui_closed)
    {
        ros::spinOnce();
        emit updateGraphics();
        loop.sleep();
    }
    emit onRosNodeFinished();
}

void QtRosNode::setNodeHandle(ros::NodeHandle* nh)
{
    this->n = nh;
}

void QtRosNode::publish_q_goal_angles(float q1, float q2, float q3, float q4, float q5, float q6, float q7)
{
    std_msgs::Float32MultiArray msg;
    msg.data.resize(7);
    msg.data[0] = q1;
    msg.data[1] = q2;
    msg.data[2] = q3;
    msg.data[3] = q4;
    msg.data[4] = q5;
    msg.data[5] = q6;
    msg.data[6] = q7;
    pubGoalQ.publish(msg);
}

void QtRosNode::publish_goal_gripper(float a)
{
}

bool QtRosNode::call_inverse_kinematics(std::vector<float>& cartesian, std::vector<float>& articular)
{
}

bool QtRosNode::call_forward_kinematics(std::vector<float>& articular, std::vector<float>& cartesian)
{
}

void QtRosNode::callback_joint_states(const sensor_msgs::JointState::ConstPtr& msg)
{
    for(int i=1; i < 8; i++)
        current_q[i-1] = msg->position[i];
}
