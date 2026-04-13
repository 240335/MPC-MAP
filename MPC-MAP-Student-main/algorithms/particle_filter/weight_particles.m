function [weights] = weight_particles(particle_measurements, lidar_distances)
%WEIGHT_PARTICLES Summary of this function goes here
%Ass3

N = size(particle_measurements, 1);
weights = zeros(N,1) / N;

sigma = 2;

for i = 1:N
    
    error = norm(particle_measurements(i,:) - lidar_distances);
    
    weights(i) = exp(-(error^2)/(2*sigma^2));
end

weights = weights / sum(weights);

end

