function plot_state(mu, sigma, z, robot_pose_hist, robot_pose_gt_hist,laser_pc_filter_in_map, mu_hist)
    % Visualizes the state of the EKF SLAM algorithm.
    %
    % The resulting plot displays the following information:
    % - map ground truth (black +'s)
    % - current robot pose estimate (red)
    % - current landmark pose estimates (blue)
    % - visualization of the observations made at this time step (line between robot and landmark)

    N_rblm = (size(mu,1)-3)/2;
    
    figure(1); clf;hold on;
    plot(nan,nan,'Color','b');
    plot(nan,nan,'Color','r');
    
    drawprobellipse(mu(1:3), sigma(1:3,1:3), 0.6, 'r');
    
    %rblm
    for i=1:N_rblm
        plot(mu(2*i+ 2),mu(2*i+ 3), 'k+', 'markersize', 5, 'linewidth', 5)
        drawprobellipse(mu(2*i+ 2:2*i+ 3), sigma(2*i+ 2:2*i+ 3,2*i+ 2:2*i+ 3), 0.6, 'b');
    end

    for i=1:size(z,2)
        mX = mu(2*z(i).id+2);
        mY = mu(2*z(i).id+3);
        line([mu(1), mX],[mu(2), mY], 'color', 'y', 'linewidth', 1);
    end

    drawrobot(mu(1:3), 'r', 3, 0.8, 0.8);
    plot(robot_pose_hist(1,:),robot_pose_hist(2,:),'Color','b');
    plot(mu_hist(1,:),mu_hist(2,:),'Color','r');
    if ~isempty(robot_pose_gt_hist)
        % drawrobot(robot_pose_gt_hist(:,end), 'g', 3, 0.3, 0.3);
        plot(robot_pose_gt_hist(1,:),robot_pose_gt_hist(2,:),'Color','g');
    end
    
%     mu_rob_M = transform_matrix_from_trans_ypr([mu(1);mu(2);0;mu(3);0;0]);
    if ~isempty(laser_pc_filter_in_map)
        plot(laser_pc_filter_in_map(:,1),laser_pc_filter_in_map(:,2),'LineStyle','none','Marker','.','Color','red');
    end
%     for n=1:size(z,2)
%         pt = [z(n).range*cos(z(n).bearing); z(n).range*sin(z(n).bearing); 0; 1];
%         pt_in_map = mu_rob_M*pt;
%         plot([mu(1), pt_in_map(1)],[mu(2), pt_in_map(2)], 'color', 'b', 'linewidth', 1)
%     end
    
    axis equal; grid on;
    legend('odom','ekf');
    xlim([-7, 7]); ylim([-7, 7]);
    xlabel('X(m)');ylabel('Y(m)');
    %filename = sprintf('../plots/ekf_%03d.png', timestep);
    %print(filename, '-dpng');
end
