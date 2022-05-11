#!/usr/bin/env python
import rospy
import numpy
from std_msgs.msg import Float32MultiArray
from control_msgs.msg import JointTrajectoryControllerState
from control_msgs.msg import GripperCommandActionGoal
from vision_msgs.srv import RecognizeObjects, RecognizeObjectsRequest, RecognizeObjectsResponse
from manip_msgs.srv import InverseKinematicsForPose, InverseKinematicsForPoseRequest, InverseKinematicsForPoseResponse
from manip_msgs.srv import ForwardKinematics, ForwardKinematicsRequest, ForwardKinematicsResponse

predefined_positions = [#[ 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000,  0.0000],
                        [-1.5000, 0.1736, 0.0000, 1.6494, 0.0000, 1.2789, -1.5002],
                        [-1.0000, 0.1736, 0.0000, 1.6494, 0.0000, 1.2789, -1.5002],
                        [-0.5000, 0.1736, 0.0000, 1.6494, 0.0000, 1.2789, -1.5002],
                        [ 0.0000, 0.1736, 0.0000, 1.6494, 0.0000, 1.2789, -1.5002],
                        [ 0.5000, 0.1736, 0.0000, 1.6494, 0.0000, 1.2789, -1.5002],
                        [ 1.0000, 0.1736, 0.0000, 1.6494, 0.0000, 1.2789, -1.5002],
                        [ 1.5000, 0.1736, 0.0000, 1.6494, 0.0000, 1.2789, -1.5002]]

def callback_state(msg):
    global goal_q
    global goal_reached
    p = numpy.asarray(msg.actual.positions)
    e = goal_q - p
    goal_reached = numpy.linalg.norm(e) < 0.05

def callback_goal_q(msg):
    global goal_q
    goal_q = numpy.asarray(msg.data)
    
def main():
    global goal_q, goal_reached
    goal_q = numpy.asarray([0,0,0,0,0,0,0])
    
    print("INITIALIZING ARM CHALLENGE PLANNING NODE...")
    rospy.init_node("arm_challenge")
    pub_goal_q = rospy.Publisher("/goal_pose", Float32MultiArray, queue_size=1)
    pub_goal_g = rospy.Publisher("/my_gen3/custom_gripper_controller/gripper_cmd/goal", GripperCommandActionGoal, queue_size=1)
    sub_state  = rospy.Subscriber("/my_gen3/gen3_joint_trajectory_controller/state", JointTrajectoryControllerState, callback_state)
    sub_goal_q = rospy.Subscriber("/goal_pose", Float32MultiArray, callback_goal_q)
    clt_recog_objects  = rospy.ServiceProxy("/recognize_objects", RecognizeObjects)
    clt_inv_kinematics = rospy.ServiceProxy("/manipulation/inverse_kinematics", InverseKinematicsForPose)
    clt_fwd_kinematics = rospy.ServiceProxy("/manipulation/forward_kinematics", ForwardKinematics)
    loop = rospy.Rate(10)

    state = "SM_INIT"
    idx   = -1
    target_object = None
    while not rospy.is_shutdown():
        if state == "SM_INIT":
            print("ActPln.->Starting state machine...")
            idx = -1
            state = "SM_SEND_NEXT_POSITION"
            
        elif state == "SM_SEND_NEXT_POSITION":
            idx = (idx + 1)%len(predefined_positions)
            print("ActPln.->Sending position " + str(idx) + " = " + str(predefined_positions[idx]))
            goal_q = numpy.asarray(predefined_positions[idx])
            msg_goal_q = Float32MultiArray()
            msg_goal_q.data = predefined_positions[idx]
            pub_goal_q.publish(msg_goal_q)
            state = "SM_WAIT_FOR_GOAL_REACHED"
            goal_reached = False
            
        elif state == "SM_WAIT_FOR_GOAL_REACHED":
            if goal_reached:
                goal_reached = False
                print("ActPln.->Goal position reached")
                state = "SM_RECOGNIZE_OBJECTS"

        elif state == "SM_RECOGNIZE_OBJECTS":
            print("ActPln.->Calling recognize objects service...")
            req = RecognizeObjectsRequest()
            resp = clt_recog_objects(req)
            print("Recognized: " + str(len(resp.recog_objects)) + " objects")
            if len(resp.recog_objects) > 0:
                zs = numpy.asarray([obj.pose.position.z for obj in resp.recog_objects])
                target_object = resp.recog_objects[numpy.argmax(zs)].pose.position
                print("Highest object at: " + str(target_object))
                state = "SM_INVERSE_KINEMATICS"
            else:
                state = "SM_SEND_NEXT_POSITION"
                
        elif state == "SM_INVERSE_KINEMATICS":
            print("Trying to calculate forward kinematics for articular position: " + str(goal_q))
            req = ForwardKinematicsRequest()
            req.q1, req.q2, req.q3, req.q4, req.q5, req.q6, req.q7 = goal_q
            resp = clt_fwd_kinematics(req)
            print("Trying to calculate inverse kinematics for pose " + str(target_object))
            req = InverseKinematicsForPoseRequest()
            req.x, req.y, req.z = target_object.x, target_object.y, target_object.z + 0.15
            req.roll, req.pitch, req.yaw = resp.roll, resp.pitch, resp.yaw
            target_object = req
            resp = clt_inv_kinematics(req)
            goal_q = numpy.asarray([resp.q1, resp.q2, resp.q3, resp.q4, resp.q5, resp.q6, resp.q7])
            print("Sending prepare position: " + str(goal_q))
            msg_goal_q = Float32MultiArray()
            msg_goal_q.data = [resp.q1, resp.q2, resp.q3, resp.q4, resp.q5, resp.q6, resp.q7]
            pub_goal_q.publish(msg_goal_q)
            state = "SM_WAIT_FOR_PREPARE_REACHED"
            goal_reached = False
            
        elif state == "SM_WAIT_FOR_PREPARE_REACHED":
            if goal_reached:
                goal_reached = False
                print("ActPln.->Prepare position reached")
                state = "SM_SEND_OBJECT_POSITION"
                
        elif state == "SM_SEND_OBJECT_POSITION":
            target_object.z -= 0.15
            print("Trying to calculate inverse kinematics for pose " + str(target_object))
            resp = clt_inv_kinematics(req)
            goal_q = numpy.asarray([resp.q1, resp.q2, resp.q3, resp.q4, resp.q5, resp.q6, resp.q7])
            print("Sending object position: " + str(goal_q))
            msg_goal_q = Float32MultiArray()
            msg_goal_q.data = [resp.q1, resp.q2, resp.q3, resp.q4, resp.q5, resp.q6, resp.q7]
            pub_goal_q.publish(msg_goal_q)
            state = "SM_WAIT_FOR_OBJECT_REACHED"
            goal_reached = False

        elif state == "SM_WAIT_FOR_OBJECT_REACHED":
            if goal_reached:
                goal_reached = False
                print("ActPln.->Prepare position reached")
                state = "SM_TAKE_OBJECT"

        elif state == "SM_TAKE_OBJECT":
            msg_grip = GripperCommandActionGoal()
            msg_grip.goal.command.position = 0.1
            pub_goal_g.publish(msg_grip)
            grip_delay_counter = 0
            state = "SM_WAIT_FOR_TAKE_OBJECT"

        elif state == "SM_WAIT_FOR_TAKE_OBJECT":
            grip_delay_counter += 1
            if grip_delay_counter > 20:
                state = "SM_LIFT_OBJECT"

        elif state == "SM_LIFT_OBJECT":
            target_object.z += 0.25
            print("Trying to calculate inverse kinematics for pose " + str(target_object))
            resp = clt_inv_kinematics(req)
            goal_q = numpy.asarray([resp.q1, resp.q2, resp.q3, resp.q4, resp.q5, resp.q6, resp.q7])
            print("Sending object position: " + str(goal_q))
            msg_goal_q = Float32MultiArray()
            msg_goal_q.data = [resp.q1, resp.q2, resp.q3, resp.q4, resp.q5, resp.q6, resp.q7]
            pub_goal_q.publish(msg_goal_q)
            state = "SM_WAIT_FOR_LIFT_OBJECT_REACHED"
            goal_reached = False

        elif state == "SM_WAIT_FOR_LIFT_OBJECT_REACHED":
            if goal_reached:
                goal_reached = False
                print("ActPln.->Prepare position reached")
                state = "SM_TAKE_OBJECT"
        loop.sleep()
    rospy.spin()

if __name__ == "__main__":
    main()
