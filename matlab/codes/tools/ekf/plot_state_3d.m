function plot_state_3d(mu, sigma, landmarks, timestep, observedLandmarks, z, robot_pose_hist, robot_pose_gt_hist)
    % Visualizes the state of the EKF SLAM algorithm.
    %
    % The resulting plot displays the following information:
    % - map ground truth (black +'s)
    % - current robot pose estimate (red)
    % - current landmark pose estimates (blue)
    % - visualization of the observations made at this time step (line between robot and landmark)

    clf
    hold on
    grid on
    L = struct2cell(landmarks); 
    %figure(1, 'visible', 'off');
    figure(1);
    %drawprobellipse(mu(1:3), sigma(1:3,1:3), 0.6, 'r');
    drawprobellipse_3d(mu(1:3), sigma(1:3,1:3), 0.997, 'r');
    plot3(cell2mat(L(2,:)), cell2mat(L(3,:)),cell2mat(L(4,:)), 'k+', 'markersize', 10, 'linewidth', 5);
    for i=1:length(observedLandmarks)
        if(observedLandmarks(i))
            plot3(mu(7+3*(i-1)+1),mu(7+3*(i-1)+2),mu(7+3*(i-1)+3), 'bo', 'markersize', 10, 'linewidth', 5)
            drawprobellipse_3d(mu(7+3*(i-1)+1:7+3*(i-1)+3), sigma(7+3*(i-1)+1:7+3*(i-1)+3,7+3*(i-1)+1:7+3*(i-1)+3), 0.997, 'b');
        end
    end

    for i=1:size(z,2)
        %mX = mu(2*z(i).id+2);
        %mY = mu(2*z(i).id+3);
        mX = mu(7+3*(z(i).id-1)+1);
        mY = mu(7+3*(z(i).id-1)+2);
        mZ = mu(7+3*(z(i).id-1)+3);
        line([mu(1), mX],[mu(2), mY],[mu(3), mZ], 'color', 'k', 'linewidth', 1);
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
