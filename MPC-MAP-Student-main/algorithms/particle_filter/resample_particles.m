function [new_particles] = resample_particles(particles, weights)
%RESAMPLE_PARTICLES Summary of this function goes here
%Ass3

N = size(particles,1);
new_particles = zeros(size(particles));

% Normalize weights just in case
weights = weights / sum(weights);

r = rand() / N;
c = weights(1);
i = 1;

for m = 1:N
    U = r + (m-1)/N;
    
    while U > c
        i = i + 1;
        c = c + weights(i);
    end
    
    new_particles(m,:) = particles(i,:);
end

new_particles(:,1:2) = new_particles(:,1:2) + randn(N,2)*0.02;

end

