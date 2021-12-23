% -------------------------------------------------------------------------
% This script has been developed as part of the research activities of
% the Automatic Control Group at UNISA
%
% Title:    Demo to generate a YZ circular path and export to ROS file
% Author:   Mario Cerino
% Org.:     UNISA - Automatic Control Group
% Date:     Dec 2020
%
% This demo generates a workspace path and exports it to the .traj format
% used in the dynamic_programming_optimization package suite. Here the path
% is a circumference lying on the y-z plane and x fixed at 1 m. The 
% end-effector orientation is constant and puts the robot's flange on the
% y-z plane as well. The script outputs the trajectory in the current
% user's home folder.
%
% -------------------------------------------------------------------------

clear all;
close all;
clc;

catkin_workspace_path = getenv('CATKIN_WORKSPACE');

filepath = strcat(catkin_workspace_path, '~/yz_circular.traj');

no_of_cp = 16;
no_of_samples = 200;

rho = 0.2;
theta = linspace(-pi, pi, no_of_cp);

ctrl_points = NaN * ones(3, no_of_cp);
ctrl_points(1,:) = 1.1 * ones(1, no_of_cp);
ctrl_points(2,:) = rho * cos(theta);
ctrl_points(3,:) = 0.8 + rho * sin(theta);

[x_lambda, lambda] = generate_path(ctrl_points, no_of_samples, true);

x_lambda(4,:) = zeros(1, no_of_samples);
x_lambda(5,:) = pi/2 * ones(1, no_of_samples);
x_lambda(6,:) = zeros(1, no_of_samples);

export_ros_workspace_path(filepath, lambda, x_lambda);