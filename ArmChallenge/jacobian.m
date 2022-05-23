function J = jacobian(robot, q)
    delta_q = 0.000001;
    J = zeros(6,7);
    qn = [q; q; q; q; q; q; q] + delta_q*eye(7);
    qp = [q; q; q; q; q; q; q] - delta_q*eye(7);
    for i=1:length(q)
        T = getTransform(robot, qn(i,:), 'EndEffector_Link', 'base_link');
        rpy = rotm2eul(T(1:3,1:3)); 
        pn  = [T(1,4) T(2,4) T(3,4) rpy(3) rpy(2) rpy(1)];
        T = getTransform(robot, qp(i,:), 'EndEffector_Link', 'base_link');
        rpy = rotm2eul(T(1:3,1:3)); 
        pp  = [T(1,4) T(2,4) T(3,4) rpy(3) rpy(2) rpy(1)];
        J(:,i) = (pn - pp)/(2*delta_q);
    end