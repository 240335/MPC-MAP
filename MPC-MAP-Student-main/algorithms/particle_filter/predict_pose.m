function [new_pose] = predict_pose(old_pose, motion_vector, read_only_vars)
%PREDICT_POSE Summary of this function goes here
%Ass3

x = old_pose(1);
y = old_pose(2);
theta = old_pose(3);

v_R = motion_vector(1);
v_L = motion_vector(2);

d  = read_only_vars.agent_drive.interwheel_dist;
dt = 0.1;

% Differential drive model
v = (v_R + v_L)/2;
omega = (v_R - v_L)/d;

% Add motion noise
sigma_v = 0.02;
sigma_w = 0.05;

v = v + randn()*sigma_v;
omega = omega + randn()*sigma_w;

% Predict new pose
x = x + v*cos(theta)*dt;
y = y + v*sin(theta)*dt;
theta = theta + omega*dt;

% Normalize angle
theta = atan2(sin(theta), cos(theta));

new_pose = [x y theta];


end

