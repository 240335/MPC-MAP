function [measurement] = compute_lidar_measurement(map, pose, lidar_config)
%COMPUTE_MEASUREMENTS Summary of this function goes here
%Ass3

x = pose(1);
y = pose(2);
theta = pose(3);

num_rays = length(lidar_config);
lidar_meas = zeros(1, num_rays);

origin = [x, y];

for i = 1:num_rays
    
    ray_dir = theta + lidar_config(i);
    
    intersections = ray_cast(origin, map.walls, ray_dir);
    
    if isempty(intersections)
        lidar_meas(i) = inf;
    else
        dists = sqrt(sum((intersections - origin).^2, 2));
        lidar_meas(i) = min(dists);
    end
end

%measurement = zeros(1, length(lidar_config));
measurement = lidar_meas;

end

