function plot_state_2d_dc(mu, sigma, landmarks,landmarks_rblm, timestep, observedLandmarks,observedLandmarks_rblm, z,z_rblm, robot_pose_hist, robot_pose_gt_hist,mu_dc,sigma_dc)
    % Visualizes the state of the EKF SLAM algorithm.
    %
    % The resulting plot displays the following information:
    % - map ground truth (black +'s)
    % - current robot pose estimate (red)
    % - current landmark pose estimates (blue)
    % - visualization of the observations made at this time step (line between robot and landmark)

    N = size(landmarks,2);
    N_rblm = size(landmarks_rblm,2);
    
    clf
    hold on
    grid on
    L = struct2cell(landmarks); 
    L_rblm = struct2cell(landmarks_rblm); 
    %figure(1, 'visible', 'off');
    figure(1);
    drawprobellipse(mu(1:3), sigma(1:3,1:3), 0.6, 'r');
    plot(cell2mat(L(2,:)), cell2mat(L(3,:)), 'k+', 'markersize', 10, 'linewidth', 5);
    for i=1:length(observedLandmarks)
        if(observedLandmarks(i))
            plot(mu_dc(2*i+ 2),mu_dc(2*i+ 3), 'bo', 'markersize', 10, 'linewidth', 5)
            drawprobellipse(mu_dc(2*i+ 2:2*i+ 3), sigma_dc(2*i+ 2:2*i+ 3,2*i+ 2:2*i+ 3), 0.997, 'b');
        end
    end
    for i=1:size(z,2)
        mX = mu_dc(2*z(i).id+2);
        mY = mu_dc(2*z(i).id+3);
        line([mu(1), mX],[mu(2), mY], 'color', 'k', 'linewidth', 1);
    end
    %rblm
    plot(cell2mat(L_rblm(2,:)), cell2mat(L_rblm(3,:)), 'm+', 'markersize', 10, 'linewidth', 5);
    for i=1:length(observedLandmarks_rblm)
        if(observedLandmarks_rblm(i))
            plot(mu(2*N+2*i+ 2),mu(2*N+2*i+ 3), 'go', 'markersize', 10, 'linewidth', 5)
            drawprobellipse(mu(2*N+2*i+ 2:2*N+2*i+ 3), sigma(2*N+2*i+ 2:2*N+2*i+ 3,2*N+2*i+ 2:2*N+2*i+ 3), 0.997, 'b');
        end
    end

    for i=1:size(z_rblm,2)
        mX = mu(2*N+2*z_rblm(i).id+2);
        mY = mu(2*N+2*z_rblm(i).id+3);
        line([mu(1), mX],[mu(2), mY], 'color', 'y', 'linewidth', 1);
    end

    drawrobot(mu(1:3), 'r', 3, 0.3, 0.3);
    plot(robot_pose_hist(1,:),robot_pose_hist(2,:),'Color','r');
    drawrobot(robot_pose_gt_hist(:,end), 'g', 3, 0.3, 0.3);
    plot(robot_pose_gt_hist(1,:),robot_pose_gt_hist(2,:),'Color','g');
    
    xlim([-2, 15])
    ylim([-2, 15])
    %filename = sprintf('../plots/ekf_%03d.png', timestep);
    %print(filename, '-dpng');
    hold off
end
