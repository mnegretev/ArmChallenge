function msg = calculate_trajectory(Q_goal, sub_current_pose)
    Q_current = receive(sub_current_pose, 3);
    Q_current = Q_current.Position(2:8);    
    speed = 0.6;
    t = 0.5 + max(abs(Q_goal - Q_current))/speed;
    
    [Qi,T] = get_polynomial_trajectory(Q_current(1), Q_goal(1), t, 0.05);
    Q = zeros(length(Q_goal), length(T));
    Q(1,:) = Qi;
    for i=2:length(Q_goal)
        [Qi,T] = get_polynomial_trajectory(Q_current(i), Q_goal(i), t, 0.05);
        Q(i,:) = Qi;
    end
    Q = Q';
    
    msg = rosmessage('trajectory_msgs/JointTrajectory', 'DataFormat', 'struct');
    msg.JointNames = {'joint_1','joint_2','joint_3','joint_4','joint_5','joint_6','joint_7'};
    for i=1:length(Q)
        p = rosmessage("trajectory_msgs/JointTrajectoryPoint", 'DataFormat', 'struct');
        p.Positions = Q(i,:);
        p.TimeFromStart = rosduration(T(i), 'DataFormat', 'struct');
        trjPts(i) = p;
    end
    msg.Points = trjPts;
    