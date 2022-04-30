#!/usr/bin/env python
import math
import rospy
import tf
import tf.transformations as tft
import numpy
import urdf_parser_py.urdf
from geometry_msgs.msg import PointStamped
from manip_msgs.srv import *

def get_model_info():
    global joints, transforms
    robot_model = urdf_parser_py.urdf.URDF.from_parameter_server(key='/my_gen3/robot_description')
    joints =  [None for i in range(8)]
    transforms =  []
    for joint in robot_model.joints:
        for i in range(1,8):
            joints[i-1] = joint if joint.name == ('joint_'+str(i)) else joints[i-1]
        joints[7] = joint if joint.name == "end_effector"       else joints[7]
    for joint in joints:
        T = tft.translation_matrix(joint.origin.xyz)
        R = tft.euler_matrix(joint.origin.rpy[0], joint.origin.rpy[1], joint.origin.rpy[2])
        transforms.append(tft.concatenate_matrices(T,R))

def angles_in_joint_limits(q):
    for i in range(len(q)):
        if q[i] < joints[i].limit.lower or q[i] > joints[i].limit.upper:
            print("InverseKinematics.->Articular position out of joint bounds")
            return False
    return True
    
def direct_kinematics(q):
    global transforms, joints
    H = tft.identity_matrix()
    print(transforms[0])
    for i in range(len(q)):
        H  = tft.concatenate_matrices(H, transforms[i], tft.rotation_matrix(q[i], joints[i].axis))
    H  = tft.concatenate_matrices(H, transforms[7])
    return numpy.asarray([H[0,3], H[1,3], H[2,3]] + list(tft.euler_from_matrix(H)))

def jacobian(q):
    delta_q = 0.000001
    J = numpy.asarray([[0.0 for a in q] for i in range(6)])##numpy, ndarray
    qn = numpy.asarray([q,]*len(q)) + delta_q*numpy.identity(len(q))
    qp = numpy.asarray([q,]*len(q)) - delta_q*numpy.identity(len(q)) 
    for i in range(len(q)):
        J[:,i] = (direct_kinematics(qn[i]) - direct_kinematics(qp[i]))/delta_q/2.0
    return J

def inverse_kinematics_xyzrpy(x, y, z, roll, pitch, yaw):
    pd = numpy.asarray([x,y,z,roll,pitch,yaw])
    q = numpy.asarray([-0.5, 0.6, 0.3, 2.0, 0.3, 0.2, 0.3])
    p  = direct_kinematics(q)
    iterations = 0
    while numpy.linalg.norm(p - pd) > 0.01 and iterations < 20:
        J = jacobian(q)
        err = p - pd
        err[3:6] = (err[3:6] + math.pi)%(2*math.pi) - math.pi
        q = (q - numpy.dot(numpy.linalg.pinv(J), err) + math.pi)%(2*math.pi) - math.pi
        p = direct_kinematics(q)
        iterations +=1
    if iterations < 20 and angles_in_joint_limits(q):
        print("InverseKinematics.->IK solved after " + str(iterations) + " iterations: " + str(q))
        return q
    else:
        print("InverseKinematics.->Cannot solve IK. Max attempts exceeded. ")
        return False

def callback_ik_for_pose(req):
    q = inverse_kinematics_xyzrpy(req.x, req.y, req.z, req.roll, req.pitch, req.yaw)
    if q is None:
        return None
    resp = InverseKinematicsForPoseResponse()
    [resp.q1, resp.q2, resp.q3, resp.q4, resp.q5, resp.q6, resp.q7] = [q[0], q[1], q[2], q[3], q[4], q[5], q[6]]
    return resp

def callback_dk(req):
    x = direct_kinematics([req.q1, req.q2, req.q3, req.q4, req.q5, req.q6, req.q7])
    resp = ForwardKinematicsResponse()
    [resp.x, resp.y, resp.z, resp.roll, resp.pitch, resp.yaw] = x
    return resp

def main():
    print("INITIALIZING INVERSE KINEMATIC NODE BY MARCOSOFT...")
    rospy.init_node("ik_geometric")
    get_model_info()
    rospy.Service("/manipulation/inverse_kinematics", InverseKinematicsForPose, callback_ik_for_pose)
    rospy.Service("/manipulation/direct_kinematics" , ForwardKinematics, callback_dk)
    loop = rospy.Rate(10)
    while not rospy.is_shutdown():
        loop.sleep()

if __name__ == '__main__':
    main()
