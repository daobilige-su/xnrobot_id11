function plot_state_3d(mu, sigma, landmarks, landmarks_rblm, timestep, observedLandmarks, observedLandmarks_rblm, z, z_rblm , robot_pose_hist, robot_pose_gt_hist)
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
    %drawprobellipse(mu(1:3), sigma(1:3,1:3), 0.6, 'r');
    drawprobellipse_3d(mu(1:3), sigma(1:3,1:3), 0.997, 'r');
    plot3(cell2mat(L(2,:)), cell2mat(L(3,:)),cell2mat(L(4,:)), 'k+', 'markersize', 10, 'linewidth', 5);
    for i=1:length(observedLandmarks)
        if(observedLandmarks(i))
            lm_global_xyz = (1/mu(7+6*(i-1)+6))*[cos(mu(7+6*(i-1)+5))*cos(mu(7+6*(i-1)+4));...
                cos(mu(7+6*(i-1)+5))*sin(mu(7+6*(i-1)+4));...
                sin(mu(7+6*(i-1)+5))]+mu(7+6*(i-1)+1:7+6*(i-1)+3);
            
            plot3(lm_global_xyz(1),lm_global_xyz(2),lm_global_xyz(3), 'bo', 'markersize', 10, 'linewidth', 5)
            
            sigma_lm = ekf_3d_jacobian_idp_to_euc(mu(7+6*(i-1)+1:7+6*(i-1)+6));
            sigma_lm = sigma_lm*sigma(7+6*(i-1)+1:7+6*(i-1)+6,7+6*(i-1)+1:7+6*(i-1)+6)*sigma_lm';
            drawprobellipse_3d(lm_global_xyz(1:3), sigma_lm, 0.997, 'b');
        end
    end

    for i=1:size(z,2)
        %mX = mu(2*z(i).id+2);
        %mY = mu(2*z(i).id+3);
%         mX = mu(7+3*(z(i).id-1)+1);
%         mY = mu(7+3*(z(i).id-1)+2);
%         mZ = mu(7+3*(z(i).id-1)+3);
        
        lm_global_xyz = (1/mu(7+6*(z(i).id-1)+6))*[cos(mu(7+6*(z(i).id-1)+5))*cos(mu(7+6*(z(i).id-1)+4));...
                cos(mu(7+6*(z(i).id-1)+5))*sin(mu(7+6*(z(i).id-1)+4));...
                sin(mu(7+6*(z(i).id-1)+5))]+mu(7+6*(z(i).id-1)+1:7+6*(z(i).id-1)+3);
        line([mu(1), lm_global_xyz(1)],[mu(2), lm_global_xyz(2)],[mu(3), lm_global_xyz(3)], 'color', 'k', 'linewidth', 1);
    end
    
    %rblm
    plot3(cell2mat(L_rblm(2,:)), cell2mat(L_rblm(3,:)), cell2mat(L_rblm(4,:)), 'm+', 'markersize', 10, 'linewidth', 5);
    for i=1:length(observedLandmarks_rblm)
        if(observedLandmarks_rblm(i))
            plot3(mu(7+6*N+3*(i-1)+ 1),mu(7+6*N+3*(i-1)+ 2),mu(7+6*N+3*(i-1)+ 3), 'go', 'markersize', 10, 'linewidth', 5)
            drawprobellipse_3d(mu(7+6*N+3*(i-1)+1:7+6*N+3*(i-1)+3), sigma(7+6*N+3*(i-1)+1:7+6*N+3*(i-1)+3,7+6*N+3*(i-1)+1:7+6*N+3*(i-1)+3), 0.997, 'b');
        end
    end

    for i=1:size(z_rblm,2)
        mX = mu(7+6*N+3*(z_rblm(i).id-1)+1);
        mY = mu(7+6*N+3*(z_rblm(i).id-1)+2);
        mZ = mu(7+6*N+3*(z_rblm(i).id-1)+3);
        line([mu(1), mX],[mu(2), mY],[mu(3), mZ], 'color', 'y', 'linewidth', 1);
    end

    drawrobot_3d(mu(1:3),'r');
    plot3(robot_pose_hist(1,:),robot_pose_hist(2,:),robot_pose_hist(3,:),'Color','r');
    drawrobot_3d(robot_pose_gt_hist(1:3,end),'g');
    plot3(robot_pose_gt_hist(1,:),robot_pose_gt_hist(2,:),robot_pose_gt_hist(3,:),'Color','g');
    xlim([-2, 15])
    ylim([-2, 15])
    %filename = sprintf('../plots/ekf_%03d.png', timestep);
    %print(filename, '-dpng');
    axis equal;
    hold off
end
