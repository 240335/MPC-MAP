function [new_mu, new_sigma] = ekf_predict(mu, sigma, u, kf, sampling_period)
%EKF_PREDICT Summary of this function goes here

dt = sampling_period;
d  = 0.2;   % wheel distance

v_R = u(1);
v_L = u(2);

% conversion v and omega
v = (v_R + v_L)/2;
omega = (v_R - v_L)/d;

theta = mu(3);

% Prediction (nonlinear model)
new_mu = [mu(1) + v*cos(theta)*dt, mu(2) + v*sin(theta)*dt, mu(3) + omega*dt];

% normaliyation
new_mu(3) = atan2(sin(new_mu(3)), cos(new_mu(3)));

% Jacobian F
F = [
    1 0 -v*sin(theta)*dt;
    0 1  v*cos(theta)*dt;
    0 0 1
];

new_sigma = F * sigma * F' + kf.R;

end

