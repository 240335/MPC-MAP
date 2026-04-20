function [new_mu, new_sigma] = kf_measure(mu, sigma, z, kf)
%KF_MEASURE Summary of this function goes here

H = [1, 0, 0;
     0, 1, 0];

K = sigma * H' / (H * sigma * H' + kf.Q); 

new_mu_col = mu(:) + K * (z(:) - H * mu(:));
new_mu = new_mu_col';

new_sigma = (eye(3) - K * H) * sigma;

end

