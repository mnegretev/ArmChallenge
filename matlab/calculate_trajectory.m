function msg = calculate_trajectory(Q_goal)
    speed = 0.4;
    m = 
    msg = rosmessage("trajectory_msgs/JointTrajectory", "DataFormat", "struct");
    msg.JointNames = {'joint_1','joint_2','joint_3','joint_4','joint_5','joint_6','joint_7'};
    for i=1:length(Q)
        p = rosmessage("trajectory_msgs/JointTrajectoryPoint", "DataFormat", "struct");
        p.Positions = Q(i);
    end