function [target] = get_target(estimated_pose, path, lookahead_dist)
%GET_TARGET Summary of this function goes here

target = [16, 2];

x = estimated_pose(1);
y = estimated_pose(2);
n = size(path, 1);

% Find the closest point on the path
dists = sqrt((path(:,1) - x).^2 + (path(:,2) - y).^2);
[~, closest_idx] = min(dists);

target = path(end, :);

for i = closest_idx : n
    d = sqrt((path(i,1) - x)^2 + (path(i,2) - y)^2);
    if d >= lookahead_dist
        target = path(i, :);
        break;
    end
end

end

