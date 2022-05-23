clear
close all
rosshutdown;
rosinit;
load('exampleHelperKINOVAGen3GripperCollRRT.mat');

predefined_positions = [ 1.5000, 0.1736, 0.0000, 1.6494, 0.0000, 1.2789, -1.5002;
                         1.0000, 0.1736, 0.0000, 1.6494, 0.0000, 1.2789, -1.5002;
                         0.5000, 0.1736, 0.0000, 1.6494, 0.0000, 1.2789, -1.5002;
                         0.0000, 0.1736, 0.0000, 1.6494, 0.0000, 1.2789, -1.5002;
                        -0.5000, 0.1736, 0.0000, 1.6494, 0.0000, 1.2789, -1.5002;
                        -1.0000, 0.1736, 0.0000, 1.6494, 0.0000, 1.2789, -1.5002;
                        -1.5000, 0.1736, 0.0000, 1.6494, 0.0000, 1.2789, -1.5002];
blue_bin_q  = [-2.1000, 0.4736, 0.0000, 1.2494, 0.0000, 1.2789, -1.5002];
green_bin_q = [ 2.1000, 0.4736, 0.0000, 1.2494, 0.0000, 1.2789, -1.5002];

predefined_positions = predefined_positions';
disp("INITIALIZING ARM CHALLENGE PLANNING NODE")

sub_current_q   = rossubscriber('/my_gen3/joint_states', 'DataFormat', 'struct');
sub_point_cloud = rossubscriber('/camera/depth/points' , 'DataFormat', 'struct');
sub_state       = rossubscriber('/my_gen3/gen3_joint_trajectory_controller/state', 'DataFormat', 'struct');
pub_goal_pose = rospublisher('/my_gen3/gen3_joint_trajectory_controller/command', 'trajectory_msgs/JointTrajectory', 'DataFormat', 'struct');
pub_goal_g    = rospublisher('/my_gen3/custom_gripper_controller/gripper_cmd/goal', 'control_msgs/GripperCommandActionGoal', 'DataFormat', 'struct');

state = "SM_INIT";
idx   = -1;
target_object = -1;
while true
    if state == "SM_INIT"
        disp("ActPln.->Starting state machine...")
        idx = -1;
        state = "SM_SEND_NEXT_POSITION";
        
    elseif state == "SM_SEND_NEXT_POSITION"
        idx = mod(idx + 1, length(predefined_positions)) + 1;
        disp("ActPln.->Sending position " + string(idx))
        goal_q = predefined_positions(:, idx);
        msg_traj = calculate_trajectory(goal_q, sub_current_q);
        send(pub_goal_pose, msg_traj);
        state = "SM_WAIT_FOR_GOAL_REACHED";
        goal_reached = 0;
            
    elseif state == "SM_WAIT_FOR_GOAL_REACHED"
        msg_state = receive(sub_state, 1);
        if norm(goal_q - msg_state.Actual.Positions) < 0.05
            disp("ActPln.->Goal position reached")
            state = "SM_RECOGNIZE_OBJECTS";
        end
            
    elseif state == "SM_RECOGNIZE_OBJECTS"
        disp("ActPln.->Calling recognize objects function...")
        [xyz, rgb] = get_cloud_wrt_base(sub_point_cloud, sub_current_q, robot);
        target_obj = get_nearest_object(xyz, rgb);
        disp("Target Object at ")
        disp(target_obj)
        if target_obj ~= [0,0,0]
            state = "SM_INVERSE_KINEMATICS";
        else
            state = "SM_SEND_NEXT_POSITION";
        end

    elseif state == "SM_INVERSE_KINEMATICS"
        disp("Trying to calculate inverse kinematics for pose " + string(target_object))
        goal_q = inverse_kinematics(double(target_obj(1)), double(target_obj(2)), double(target_obj(3) + 0.15), robot, sub_current_q);
        disp("Sending prepare position: " + string(goal_q))
        send(pub_goal_pose, calculate_trajectory(goal_q, sub_current_q));
        state = "SM_WAIT_FOR_PREPARE_REACHED";

    elseif state == "SM_WAIT_FOR_PREPARE_REACHED"
        msg_state = receive(sub_state, 1);
        if norm(goal_q - msg_state.Actual.Positions) < 0.05
            disp("ActPln.->Prepare position reached")
            state = "SM_SEND_OBJECT_POSITION";
        end

    elseif state == "SM_SEND_OBJECT_POSITION"
        disp("Trying to calculate inverse kinematics for pose " + string(target_obj))
        goal_q = inverse_kinematics(double(target_obj(1)), double(target_obj(2)), double(target_obj(3)), robot, sub_current_q);
        disp("Sending object position: " + string(goal_q))
        send(pub_goal_pose, calculate_trajectory(goal_q, sub_current_q));
        state = "SM_WAIT_FOR_OBJECT_REACHED";
        counter = 0;

    elseif state == "SM_WAIT_FOR_OBJECT_REACHED"
        msg_state = receive(sub_state, 1);
        counter = counter + 1;
        if norm(goal_q - msg_state.Actual.Positions) < 0.05 || counter > 30
            disp("ActPln.->Object position reached")
            state = "SM_TAKE_OBJECT";
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
        disp("Trying to calculate inverse kinematics for pose " + string(target_object))
        goal_q = inverse_kinematics(double(target_obj(1)), double(target_obj(2)), double(target_obj(3) + 0.25), robot, sub_current_q);
        disp("Sending object position: " + string(goal_q))
        send(pub_goal_pose, calculate_trajectory(goal_q, sub_current_q));
        state = "SM_WAIT_FOR_LIFT_OBJECT_REACHED";
        
    elseif state == "SM_WAIT_FOR_LIFT_OBJECT_REACHED"
        msg_state = receive(sub_state, 1);
        if norm(goal_q - msg_state.Actual.Positions) < 0.05
            disp("ActPln.->Lift Object position reached")
            state = "SM_MOVE_TO_BIN";
        end

    elseif state == "SM_MOVE_TO_BIN"
        goal_q = green_bin_q';
        disp("Sending object position: " + string(goal_q))
        msg_traj = calculate_trajectory(goal_q, sub_current_q);
        send(pub_goal_pose, msg_traj);
        state = "SM_WAIT_FOR_MOVE_TO_BIN";

    elseif state == "SM_WAIT_FOR_MOVE_TO_BIN"
        msg_state = receive(sub_state, 1);
        if norm(goal_q - msg_state.Actual.Positions) < 0.05
            disp("ActPln.->Bin position reached")
            state = "SM_LEAVE_OBJECT";
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
