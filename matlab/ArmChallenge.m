clear
close all
rosshutdown;
rosinit;
load('exampleHelperKINOVAGen3GripperCollRRT.mat');
predefined_positions = [-1.5000, 0.1736, 0.0000, 1.6494, 0.0000, 1.2789, -1.5002;
                        -1.0000, 0.1736, 0.0000, 1.6494, 0.0000, 1.2789, -1.5002;
                        -0.5000, 0.1736, 0.0000, 1.6494, 0.0000, 1.2789, -1.5002;
                         0.0000, 0.1736, 0.0000, 1.6494, 0.0000, 1.2789, -1.5002;
                         0.5000, 0.1736, 0.0000, 1.6494, 0.0000, 1.2789, -1.5002;
                         1.0000, 0.1736, 0.0000, 1.6494, 0.0000, 1.2789, -1.5002;
                         1.5000, 0.1736, 0.0000, 1.6494, 0.0000, 1.2789, -1.5002];
predefined_positions = predefined_positions';
disp("INITIALIZING ARM CHALLENGE PLANNING NODE")

sub_current_q   = rossubscriber('/my_gen3/joint_states', 'DataFormat', 'struct');
sub_point_cloud = rossubscriber('/camera/depth/points' , 'DataFormat', 'struct');
pub_goal_pose = rospublisher('/my_gen3/gen3_joint_trajectory_controller/command', 'trajectory_msgs/JointTrajectory', 'DataFormat', 'struct');
