function [roll, pitch, yaw] = get_best_grasping_orientation(sub_current_q, robot, obj_axis)
    %% Get current configuration
    q = receive(sub_current_q, 1);
    q = q.Position(2:8);
    T = getTransform(robot, q', 'EndEffector_Link', 'base_link');
    rpy = rotm2eul(T(1:3,1:3)); 
    current_yaw   = rpy(1);
    %%
    roll = 3.1;
    pitch = 0.0;
    [m, idx] = max(obj_axis(:,1));
    if idx == 3
        yaw = current_yaw;
    else
        yaw = atan(obj_axis(2,1)/obj_axis(1,1)) + 1.5708;
    end