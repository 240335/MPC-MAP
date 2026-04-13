function [public_vars] = init_particle_filter(read_only_vars, public_vars)
%INIT_PARTICLE_FILTER Summary of this function goes here
N = 1000;

x_min = 0;
x_max = 10;

y_min = 0;
y_max = 10;

public_vars.particles = zeros(N,3);

public_vars.particles(:,1) = x_min + rand(N,1)*(x_max - x_min);
public_vars.particles(:,2) = y_min + rand(N,1)*(y_max - y_min);
public_vars.particles(:,3) = -pi + rand(N,1)*2*pi;

end

