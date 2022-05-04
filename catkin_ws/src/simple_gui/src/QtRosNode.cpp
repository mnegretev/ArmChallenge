#include "QtRosNode.h"

QtRosNode::QtRosNode()
{
    this->gui_closed = false;
    this->current_q.resize(7);
    this->current_cartesian.resize(6);
}

QtRosNode::~QtRosNode()
{
}

void QtRosNode::run()
{    
    ros::Rate loop(10);
    subJointStates = n->subscribe("/my_gen3/joint_states", 1, &QtRosNode::callback_joint_states, this);
    pubGoalQ    = n->advertise<std_msgs::Float32MultiArray>("/goal_pose", 10);
    pubGoalGrip = n->advertise<control_msgs::GripperCommandActionGoal>("/my_gen3/custom_gripper_controller/gripper_cmd/goal", 10);
    cltForwardKinematics = n->serviceClient<manip_msgs::ForwardKinematics>("/manipulation/forward_kinematics");
    cltInverseKinematics = n->serviceClient<manip_msgs::InverseKinematicsForPose>("/manipulation/inverse_kinematics");
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
    control_msgs::GripperCommandActionGoal msg;
    msg.goal.command.position = a;
    pubGoalGrip.publish(msg);
}

bool QtRosNode::call_inverse_kinematics(std::vector<float>& cartesian, std::vector<float>& articular)
{
    articular = current_q;
    manip_msgs::InverseKinematicsForPose srv;
    srv.request.x = cartesian[0];
    srv.request.y = cartesian[1];
    srv.request.z = cartesian[2];
    srv.request.roll  = cartesian[3];
    srv.request.pitch = cartesian[4];
    srv.request.yaw   = cartesian[5];
    if(!cltInverseKinematics.call(srv))
        return false;
    articular[0] = srv.response.q1;
    articular[1] = srv.response.q2;
    articular[2] = srv.response.q3;
    articular[3] = srv.response.q4;
    articular[4] = srv.response.q5;
    articular[5] = srv.response.q6;
    articular[6] = srv.response.q7;
    return true;
}

bool QtRosNode::call_forward_kinematics(std::vector<float>& articular, std::vector<float>& cartesian)
{
    cartesian.resize(6);
    for(int i=0; i<cartesian.size(); i++) cartesian[i] = 0;
    manip_msgs::ForwardKinematics srv;
    srv.request.q1 = articular[0];
    srv.request.q2 = articular[1];
    srv.request.q3 = articular[2];
    srv.request.q4 = articular[3];
    srv.request.q5 = articular[4];
    srv.request.q6 = articular[5];
    srv.request.q7 = articular[6];
    if(!cltForwardKinematics.call(srv))
        return false;
    cartesian[0] = srv.response.x;
    cartesian[1] = srv.response.y;
    cartesian[2] = srv.response.z;
    cartesian[3] = srv.response.roll;
    cartesian[4] = srv.response.pitch;
    cartesian[5] = srv.response.yaw;
    return true;
}

void QtRosNode::callback_joint_states(const sensor_msgs::JointState::ConstPtr& msg)
{
    for(int i=1; i < 8; i++)
        current_q[i-1] = msg->position[i];
    call_forward_kinematics(current_q, current_cartesian);
}
