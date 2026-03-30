function [public_vars] = student_workspace(read_only_vars,public_vars)
%STUDENT_WORKSPACE Summary of this function goes here

% 8. Perform initialization procedure
if (read_only_vars.counter == 1)
          
    public_vars = init_particle_filter(read_only_vars, public_vars);
    public_vars = init_kalman_filter(read_only_vars, public_vars);

end

% 9. Update particle filter
public_vars.particles = update_particle_filter(read_only_vars, public_vars);

% 10. Update Kalman filter
[public_vars.mu, public_vars.sigma] = update_kalman_filter(read_only_vars, public_vars);

% 11. Estimate current robot position
public_vars.estimated_pose = estimate_pose(public_vars); % (x,y,theta)

% 12. Path planning
public_vars.path = plan_path(read_only_vars, public_vars);

% 13. Plan next motion command
public_vars = plan_motion(read_only_vars, public_vars);


% My code - Assig 1
if read_only_vars.counter <= 100    % Sber 100 vzorku pro vypocet 
    public_vars.motion_vector = [0, 0];
    public_vars.pom_prom1 = true;
    public_vars.gnss_meas_vals(1, read_only_vars.counter) = read_only_vars.gnss_position(1);
    public_vars.gnss_meas_vals(2, read_only_vars.counter) = read_only_vars.gnss_position(2);
    
    for i = 1:8
        public_vars.lidar_meas_vals(i, read_only_vars.counter) = read_only_vars.lidar_distances(i);
    end  
end

if read_only_vars.counter > 100 && public_vars.pom_prom1 == true % Provedeni vypoctu a vykresleni grafu
    public_vars.pom_prom1 = false;

    for i = 1:2     % Std a Histogram GNSS
        public_vars.std_gnss(1, i) = std(public_vars.gnss_meas_vals(1,:));
        figure(2);
        subplot(2,1,i);
        histogram(public_vars.gnss_meas_vals(i,:));
        title('GNSS ch. ', num2str(i));
        xlabel('Distance');
        ylabel('No. of samples');
    end
    
    for i = 1:8     % Std a Histogram Lidaru
        public_vars.std_lidar(1, i) = std(public_vars.lidar_meas_vals(i,:));
        figure(3);
        subplot(4,2,i);
        histogram(public_vars.lidar_meas_vals(i,:));
        title('LIDAR ch. ', num2str(i));
        xlabel('Distance');
        ylabel('No. of samples');
        grid on;
    end

    lidar_cov_mat = cov(public_vars.lidar_meas_vals')
    gnss_cov_mat = cov(public_vars.gnss_meas_vals')

    % Vypocet pdf
    mu=0;
    x_lidar = linspace(mu - 3*public_vars.std_lidar(1,8), mu + 3*public_vars.std_lidar(1,8), 1000);
    x_gnss = linspace(mu - 3*public_vars.std_gnss(1,1), mu + 3*public_vars.std_gnss(1,1), 1000);

    pdf_lidar = norm_pdf(x_lidar, mu, public_vars.std_lidar(1,8));
    pdf_gnss = norm_pdf(x_gnss, mu, public_vars.std_gnss(1,1));
    
    % Vykresleni pdf
    figure(4);
    plot(x_lidar, pdf_lidar, 'b', 'LineWidth', 2); hold on;
    plot(x_gnss, pdf_gnss, 'r', 'LineWidth', 2);
    xlabel('Error [m]');
    ylabel('Probability Density');
    legend('LiDAR 8th Ch.', 'GNSS X-axis'); 
    grid on;
    title('Sensor Noise Characteristics (Normal Distribution)');

end

end

