function [public_vars] = plan_motion(read_only_vars, public_vars)
%PLAN_MOTION Summary of this function goes here

% I. Pick navigation target

target = get_target(public_vars.estimated_pose, public_vars.path);

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

end