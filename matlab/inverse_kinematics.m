function Q = inverse_kinematics(x,y,z, robot, sub_current_pose)
    Q = receive(sub_current_pose, 1);
    Q = Q.Position(2:8);
    
    %%% Get Forward Kinematics for current configuration
    T = getTransform(robot, Q', 'EndEffector_Link', 'base_link');
    rpy = rotm2eul(T(1:3,1:3));
    p  = [T(1,4) T(2,4) T(3,4) rpy(3) rpy(2) rpy(1)];
    pd = [x y z p(4) p(5) p(6)];
    iterations = 0;
    while norm(p - pd) > 0.01 && iterations < 200
        J = jacobian(robot, Q);
        err = p - pd;
        err(4:6) = mod(err(4:6) + pi, 2*pi) - pi;
        Q = mod(q - pinv(J)*err + pi, 2*pi) - pi;
        T = getTransform(robot, Q', 'EndEffector_Link', 'base_link');
        rpy = rotm2eul(T(1:3,1:3));
        p  = [T(1,4) T(2,4) T(3,4) rpy(3) rpy(2) rpy(1)];
        iterations = iterations + 1;
    end
    
    if iterations >= 20
        Q = [0];
    end
        