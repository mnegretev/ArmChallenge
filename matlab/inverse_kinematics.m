function Q = inverse_kinematics(x,y,z, robot, sub_current_pose)
    Q_current = receive(sub_current_pose, 1);
    Q_current = Q_current.Position(2:8);
    
    %%% Get Forward Kinematics for current configuration
    T = getTransform(robot, Q_current', 'EndEffector_Link', 'base_link');
    rpy = rotm2eul(T(1:3,1:3));
    p  = [T(1,4) T(2,4) T(3,4) rpy(3) rpy(2) rpy(1)];
    pd = [x y z p(4) p(5) p(6)];
    