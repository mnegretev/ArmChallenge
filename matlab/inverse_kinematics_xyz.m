function [success, Q] = inverse_kinematics_xyz(pd, robot, sub_current_pose)
    Q = receive(sub_current_pose, 3);
    Q = Q.Position(2:8);
    success = true;
    
    %% Get Forward Kinematics for current configuration
    T = getTransform(robot, Q', 'EndEffector_Link', 'base_link');
    p  = [T(1,4) T(2,4) T(3,4)];
    iterations = 0;
    while norm(p - pd) > 0.01 && iterations < 200
        J = jacobian(robot, Q');
        J = J(1:3,:);
        err = p - pd;
        Q = mod(Q - pinv(J)*err' + pi, 2*pi) - pi;
        T = getTransform(robot, Q', 'EndEffector_Link', 'base_link');
        p  = [T(1,4) T(2,4) T(3,4)];
        iterations = iterations + 1;
    end
    
    if iterations >= 20
        success = false;
    end
        