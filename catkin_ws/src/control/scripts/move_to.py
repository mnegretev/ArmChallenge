#!/usr/bin/env python
import rospy
import numpy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def get_current_position():
    msg = rospy.wait_for_message("/my_gen3/joint_states", JointState)
    q = msg.position[1:8]
    return q

def get_q_polynomial_trajectory(q0, q1, t, dt):
    A = [[   t**5,    t**4,   t**3, t**2, t, 1],
         [ 5*t**4,  4*t**3, 3*t**2,  2*t, 1, 0],
         [20*t**3, 12*t**2,    6*t,    2, 0, 0],
         [      0,       0,      0,    0, 0, 1],
         [      0,       0,      0,    0, 1, 0],
         [      0,       0,      0,    2, 0, 0]]
    A = numpy.asarray(A)
    B = [[q1],
         [ 0],
         [ 0],
         [q0],
         [ 0],
         [ 0]]
    B = numpy.asarray(B)
    X = numpy.dot(numpy.linalg.inv(A),B)
    a5, a4, a3, a2, a1, a0 = X[0,0], X[1,0], X[2,0], X[3,0], X[4,0], X[5,0]
    T = numpy.arange(0, t, dt)
    Q = numpy.zeros(len(T))
    for i in range(len(T)):
        Q[i] = a5*T[i]**5 + a4*T[i]**4 + a3*T[i]**3 + a2*T[i]**2 + a1*T[i] + a0

    return T, Q

def get_trajectory(q0, q1, t, dt):
    Q = []
    T = []
    for i in range(len(q0)):
        T, Qi = get_q_polynomial_trajectory(q0[i], q1[i], t, dt)
        Q.append(Qi)
    Q = numpy.asarray(Q)
    Q = Q.transpose()
    trj = JointTrajectory()
    trj.header.stamp = rospy.Time.now()
    trj.joint_names = ["joint_1","joint_2","joint_3","joint_4","joint_5","joint_6","joint_7"]
    for i in range(len(Q)):
        p = JointTrajectoryPoint()
        p.positions = Q[i]
        p.time_from_start = rospy.Duration.from_sec(T[i])
        trj.points.append(p)
    return trj

def get_good_time(q0, q1):
    speed = 0.4
    m = max([abs(q0[i]-q1[i]) for i in range(len(q0))])
    return 0.5 + m/speed

def callback_goal_pose(msg):
    global trj_publisher
    q0 = get_current_position()
    q1 = msg.data
    trj_publisher.publish(get_trajectory(q0, q1, get_good_time(q0, q1), 0.05))

def main():
    global trj_publisher
    print("INITIALIZING ARM CONTROL...")
    rospy.init_node("arm_control")
    loop = rospy.Rate(10)
    trj_publisher = rospy.Publisher('/my_gen3/gen3_joint_trajectory_controller/command', JointTrajectory, queue_size=10)
    rospy.Subscriber("/goal_pose", Float32MultiArray, callback_goal_pose)
    
    while not rospy.is_shutdown():
        loop.sleep()
    

if __name__ == "__main__":
    main()
