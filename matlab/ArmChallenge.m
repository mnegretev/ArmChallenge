clear
close all
rosshutdown;
rosinit;
load('exampleHelperKINOVAGen3GripperCollRRT.mat');

predefined_positions = [-1.50, -0.50, 0.00, 2.20,  0.00, 1.00, -1.50;
                        -1.00, -0.50, 0.00, 2.20,  0.00, 1.00, -1.50;
                        -0.50, -0.50, 0.00, 2.20,  0.00, 1.00, -1.50;
                         0.00, -0.50, 0.00, 2.20,  0.00, 1.00, -1.50;
                         0.50, -0.50, 0.00, 2.20,  0.00, 1.00, -1.50;
                         1.00, -0.50, 0.00, 2.20,  0.00, 1.00, -1.50;
                         1.50, -0.50, 0.00, 2.20,  0.00, 1.00, -1.50;
                         1.50,  0.18, 0.00, 1.95, -0.10, 0.50, -1.50;
                         1.00,  0.18, 0.00, 1.95, -0.10, 0.50, -1.50;
                         0.50,  0.18, 0.00, 1.95, -0.10, 0.50, -1.50;
                         0.00,  0.18, 0.00, 1.95, -0.10, 0.50, -1.50;
                        -1.00,  0.18, 0.00, 1.95, -0.10, 0.50, -1.50;
                        -0.20,  0.50, 0.00, 1.50,  0.00, 0.60, -1.50;
                         0.00,  0.50, 0.00, 1.50,  0.00, 0.60, -1.50;
                         0.20,  0.50, 0.00, 1.50,  0.00, 0.60, -1.50;
                         0.40,  0.50, 0.00, 1.50,  0.00, 0.60, -1.50;];
green_bin_q = [-2.1000, 0.4736, 0.0000, 1.2494, 0.0000, 1.2789, -1.5002];
blue_bin_q  = [ 2.1000, 0.4736, 0.0000, 1.2494, 0.0000, 1.2789, -1.5002];

predefined_positions = predefined_positions';
disp("INITIALIZING ARM CHALLENGE PLANNING NODE")

sub_current_q   = rossubscriber('/my_gen3/joint_states', 'DataFormat', 'struct');
sub_point_cloud = rossubscriber('/camera/depth/points' , 'DataFormat', 'struct');
sub_state       = rossubscriber('/my_gen3/gen3_joint_trajectory_controller/state', 'DataFormat', 'struct');
pub_goal_pose = rospublisher('/my_gen3/gen3_joint_trajectory_controller/command', 'trajectory_msgs/JointTrajectory', 'DataFormat', 'struct');
pub_goal_g    = rospublisher('/my_gen3/custom_gripper_controller/gripper_cmd/goal', 'control_msgs/GripperCommandActionGoal', 'DataFormat', 'struct');

state = "SM_INIT";
idx   = -1;
obj_position = [inf inf inf];
obj_class = 'unknown';
obj_axis  = zeros(3,3);
while true
    if state == "SM_INIT"
        disp("ActPln.->Starting state machine...")
        idx = -1;
        state = "SM_SEND_NEXT_POSITION";
        
    elseif state == "SM_SEND_NEXT_POSITION"
        idx = mod(idx + 1, length(predefined_positions));
        goal_q = predefined_positions(:, idx+1);
        disp("ActPln.->Sending position: " + num2str(idx+1) + "=" + mat2str(goal_q'))
        msg_traj = calculate_trajectory(goal_q, sub_current_q);
        send(pub_goal_pose, msg_traj);
        state = "SM_WAIT_FOR_GOAL_REACHED";
        goal_reached = 0;
            
    elseif state == "SM_WAIT_FOR_GOAL_REACHED"
        msg_state = receive(sub_state, 3);
        if norm(goal_q - msg_state.Actual.Positions) < 0.05
            disp("ActPln.->Goal position reached")
            state = "SM_RECOGNIZE_OBJECTS";
            pause(1.0);
        end
            
    elseif state == "SM_RECOGNIZE_OBJECTS"
        disp("ActPln.->Calling recognize objects function...")
        [obj_position, obj_class, obj_axis] = get_nearest_object(sub_point_cloud, sub_current_q, robot);
        if obj_position == [inf inf inf]
            state = "SM_SEND_NEXT_POSITION";
            disp("Cannot find known object. Moving to next position")
        else
            state = "SM_INVERSE_KINEMATICS";
            disp("Target Object at " + mat2str(obj_position))
        end

    elseif state == "SM_INVERSE_KINEMATICS"
        [roll,pitch,yaw] = get_best_grasping_orientation(sub_current_q, robot, obj_axis);
        goal_cartesian = [obj_position roll pitch yaw] + [0 0 0.15 0 0 0];
        disp("Trying to calculate inverse kinematics for pose " + mat2str(goal_cartesian))
        [success, goal_q] = inverse_kinematics(goal_cartesian, robot, sub_current_q);
        if ~success
            disp("Cannot calculate inverse kinematics for full configuration. Trying only for position...")
            goal_cartesian = obj_position + [0 0 0.15];
            [success, goal_q] = inverse_kinematics_xyz(goal_cartesian, robot, sub_current_q);
        end
        if ~success
            disp("Cannot calculate inverse kinematics for object position. Moving to next position.")
            state = "SM_SEND_NEXT_POSITION";
        else
            disp("Sending prepare position: " + mat2str(goal_q'))
            send(pub_goal_pose, calculate_trajectory(goal_q, sub_current_q));
            state = "SM_WAIT_FOR_PREPARE_REACHED";
        end

    elseif state == "SM_WAIT_FOR_PREPARE_REACHED"
        msg_state = receive(sub_state, 3);
        if norm(goal_q - msg_state.Actual.Positions) < 0.05
            disp("ActPln.->Prepare position reached")
            state = "SM_SEND_OBJECT_POSITION";
            pause(1.0)
        end

    elseif state == "SM_SEND_OBJECT_POSITION"
        goal_cartesian = [obj_position roll pitch yaw];
        disp("Trying to calculate inverse kinematics for pose " + mat2str(goal_cartesian))
        [success, goal_q] = inverse_kinematics(goal_cartesian, robot, sub_current_q);
        if ~success
            disp("Cannot calculate inverse kinematics for full configuration. Trying only for position...")
            [success, goal_q] = inverse_kinematics_xyz(obj_position, robot, sub_current_q);
        end
        if ~success
            disp("Cannot calculate inverse kinematics for object position. Moving to next position.")
            state = "SM_SEND_NEXT_POSITION";
        else
            disp("Sending object position: " + mat2str(goal_q'))
            send(pub_goal_pose, calculate_trajectory(goal_q, sub_current_q));
            state = "SM_WAIT_FOR_OBJECT_REACHED";
            counter = 0;
        end

    elseif state == "SM_WAIT_FOR_OBJECT_REACHED"
        msg_state = receive(sub_state, 3);
        counter = counter + 1;
        if norm(goal_q - msg_state.Actual.Positions) < 0.05 || counter > 30
            disp("ActPln.->Object position reached")
            state = "SM_TAKE_OBJECT";
            pause(1.0)
        end

    elseif state == "SM_TAKE_OBJECT"
        msg_grip = rosmessage('control_msgs/GripperCommandActionGoal', 'DataFormat', 'struct');
        msg_grip.Goal.Command.Position = 0.1;
        send(pub_goal_g, msg_grip);
        grip_delay_counter = 0;
        state = "SM_WAIT_FOR_TAKE_OBJECT";

    elseif state == "SM_WAIT_FOR_TAKE_OBJECT"
        grip_delay_counter = grip_delay_counter + 1;
        if grip_delay_counter > 20
            state = "SM_LIFT_OBJECT";
        end

    elseif state == "SM_LIFT_OBJECT"
        goal_cartesian = [obj_position roll pitch yaw] + [0 0 0.35 0 0 0];
        disp("Trying to calculate inverse kinematics for pose " + mat2str(goal_cartesian))
        [success, goal_q] = inverse_kinematics(goal_cartesian, robot, sub_current_q);
        if ~success
            disp("Cannot calculate inverse kinematics for full configuration. Trying only for position...")
            goal_cartesian = obj_position + [0 0 0.15];
            [success, goal_q] = inverse_kinematics_xyz(goal_cartesian, robot, sub_current_q);
        end
        if ~success
            disp("Cannot calculate inverse kinematics for object position. Moving to next position.")
            state = "SM_SEND_NEXT_POSITION";
        else
            disp("Sending object position: " + mat2str(goal_q'))
            send(pub_goal_pose, calculate_trajectory(goal_q, sub_current_q));
            state = "SM_WAIT_FOR_LIFT_OBJECT_REACHED";
        end
        
    elseif state == "SM_WAIT_FOR_LIFT_OBJECT_REACHED"
        msg_state = receive(sub_state, 3);
        if norm(goal_q - msg_state.Actual.Positions) < 0.05
            disp("ActPln.->Lift Object position reached")
            state = "SM_MOVE_TO_BIN";
        end

    elseif state == "SM_MOVE_TO_BIN"
        if strcmp(obj_class, 'can')
            disp("Found a can object. Moving to green bin")
            goal_q = green_bin_q';
            msg_traj = calculate_trajectory(goal_q, sub_current_q);
            send(pub_goal_pose, msg_traj);
            state = "SM_WAIT_FOR_MOVE_TO_BIN";
        elseif strcmp(obj_class, 'bottle')
            disp("Found a bottle object. Moving to blue bin")
            goal_q = blue_bin_q';
            msg_traj = calculate_trajectory(goal_q, sub_current_q);
            send(pub_goal_pose, msg_traj);
            state = "SM_WAIT_FOR_MOVE_TO_BIN";
        else
            disp("Found an unknown object. Moving to next position")
            state = "SM_SEND_NEXT_POSITION";
        end
        

    elseif state == "SM_WAIT_FOR_MOVE_TO_BIN"
        msg_state = receive(sub_state, 3);
        if norm(goal_q - msg_state.Actual.Positions) < 0.05
            disp("ActPln.->Bin position reached")
            state = "SM_LEAVE_OBJECT";
            pause(1.0)
        end

    elseif state == "SM_LEAVE_OBJECT"
        msg_grip = rosmessage("control_msgs/GripperCommandActionGoal", "DataFormat", "struct");
        msg_grip.Goal.Command.Position = 0.0;
        send(pub_goal_g, msg_grip);
        grip_delay_counter = 0;
        state = "SM_WAIT_FOR_LEAVE_OBJECT";
        
    elseif state == "SM_WAIT_FOR_LEAVE_OBJECT"
        grip_delay_counter = grip_delay_counter + 1;
        if grip_delay_counter > 20
            state = "SM_SEND_NEXT_POSITION";
        end
        
    end
    drawnow
    pause(0.1);
end
