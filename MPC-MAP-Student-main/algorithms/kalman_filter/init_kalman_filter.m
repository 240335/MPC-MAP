function [public_vars] = init_kalman_filter(read_only_vars, public_vars)
%INIT_KALMAN_FILTER Summary of this function goes here

public_vars.kf.C = ones(2);
public_vars.kf.R = diag([0.0001 0.0001 0.00005]);
public_vars.kf.Q = public_vars.gnss_cov_mat;

public_vars.mu = [public_vars.gnss_means(1), public_vars.gnss_means(2), 0];
public_vars.sigma = zeros(3,3); %diag([0.5, 0.5, 0.25]);

end

