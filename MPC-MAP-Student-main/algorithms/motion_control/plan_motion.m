function [public_vars] = plan_motion(read_only_vars, public_vars)
%PLAN_MOTION Summary of this function goes here

% I. Pick navigation target

%target = get_target(public_vars.estimated_pose, public_vars.path,);

%{
if public_vars.help_cnt > 0 && public_vars.help_cnt <= 100
    public_vars.motion_vector = [0, 0];
elseif public_vars.help_cnt > 100 && public_vars.help_cnt <= 300
    public_vars.motion_vector = [0.3, 0.302];
elseif public_vars.help_cnt > 300 && public_vars.help_cnt <= 500
    public_vars.motion_vector = [0.3, 0.332];
elseif public_vars.help_cnt > 500 && public_vars.help_cnt <= 600
    public_vars.motion_vector = [0.3, 0.3];
elseif public_vars.help_cnt > 600 && public_vars.help_cnt <= 780
    public_vars.motion_vector = [0.334, 0.3];
elseif public_vars.help_cnt > 780 && public_vars.help_cnt <= 1000
    public_vars.motion_vector = [0.3, 0.3];
else
    public_vars.motion_vector = [0, 0];
end

public_vars.help_cnt = public_vars.help_cnt + 1;
%}

% Tuning parameters – vary these to observe their effect (see report)
LOOKAHEAD_DIST = 1.0;   % (m)   look-ahead distance L                           
LINEAR_VEL     = 0.35;  % (m/s) constant forward speed            
GOAL_RADIUS    = 0.25;  % (m)   stop when within this distance of path end

d       = read_only_vars.agent_drive.interwheel_dist;   % 0.2 m
max_vel = read_only_vars.agent_drive.max_vel;           % 1.0 m/s

% Retrieve MoCap pose
pose = read_only_vars.mocap_pose;   % [x, y, theta]  (NaN outside MoCap zone)
path = public_vars.path;            % N×2 waypoint matrix

% Safety: no pose or no path  → stand still
if isempty(pose) || any(isnan(pose)) || isempty(path) || size(path,1) < 2
    public_vars.motion_vector = [0, 0];
    return;
end

x     = pose(1);
y     = pose(2);
theta = pose(3);

% --- Stop condition: robot close to the last waypoint ------------------
dist_to_end = sqrt((path(end,1) - x)^2 + (path(end,2) - y)^2);
if dist_to_end < GOAL_RADIUS
    public_vars.motion_vector = [0, 0];
    return;
end

% Pure Pursuit
target = get_target(pose, path, LOOKAHEAD_DIST);    % look-ahead point [x, y]

% Heading angle from robot to target (in robot-fixed frame)
alpha = atan2(target(2) - y, target(1) - x) - theta;
alpha = atan2(sin(alpha), cos(alpha));              % normalise to [-π, π]

% Arc curvature: κ = 2·sin(α) / L
kappa = 2 * sin(alpha) / LOOKAHEAD_DIST;

% Differential-drive conversion
omega = LINEAR_VEL * kappa;
v_R   = LINEAR_VEL + omega * d / 2;
v_L   = LINEAR_VEL - omega * d / 2;

% Clamp to hardware limits
v_R = max(-max_vel, min(max_vel, v_R));
v_L = max(-max_vel, min(max_vel, v_L));

public_vars.motion_vector = [v_R, v_L];

end