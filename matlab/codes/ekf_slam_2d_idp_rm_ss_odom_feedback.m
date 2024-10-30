% This is the main extended Kalman filter SLAM loop. This script calls all the required
% functions in the correct order.
%
% You can disable the plotting or change the number of steps the filter
% runs for to ease the debugging. You should however not change the order
% or calls of any of the other lines, as it might break the framework.
%
% If you are unsure about the input and return values of functions you
% should read their documentation which tells you the expected dimensions.

% Turn off pagination:
more off;
close all; clear;

% Make tools available
addpath('tools/ekf_idp');

% % Read world data, i.e. landmarks. The true landmark positions are not given to the robot
% landmarks = read_world_2d('../data/world_2d.dat');
% % Read sensor readings, i.e. odometry and range-bearing sensor
% data = read_data_2d('../data/sensor_data_2d.dat');
% % Read other supplementary information
%load ../data/sup_data_2d.mat
load ../data/mic_array_rblm_2d.mat

INF = 100000;
% Get the number of landmarks in the map
N = size(landmarks,2);
N_rblm = size(landmarks_rblm,2);

% observedLandmarks is a vector that keeps track of which landmarks have been observed so far.
% observedLandmarks(i) will be true if the landmark with id = i has been observed at some point by the robot
observedLandmarks = false(1,N);
observedLandmarks_rblm = false(1,N_rblm);

% Initialize belief:
% mu: 2N+3x1 vector representing the mean of the normal distribution
% The first 3 components of mu correspond to the pose of the robot,
% and the landmark poses (xi, yi) are stacked in ascending id order.
% sigma: (2N+3)x(2N+3) covariance matrix of the normal distribution
mu = repmat([0.0], (4*N+2*N_rblm+3), 1);
mu(1:3) = [0.2;0;0];
robSigma = zeros(3);
robMapSigma = zeros(3,4*N+2*N_rblm);
mapSigma = INF*eye(4*N+2*N_rblm);
sigma = [[robSigma robMapSigma];[robMapSigma' mapSigma]];

% Perform filter update for each odometry-observation pair read from the
% data file.
robot_pose_hist = [];
robot_pose_gt_hist = [];
robot_pose_gt = [];

for t = 1:size(data.timestep, 2)
    
    robot_pose_gt = sup_data.robot_pose_gt(t+1,:)';
    robot_pose_gt_hist = [robot_pose_gt_hist,robot_pose_gt];

    % Perform the prediction step of the EKF
    if t>1
        [mu, sigma] = prediction_step_2d(mu, sigma, data.timestep(t).odometry, sup_data.u_dist_std, sup_data.u_theta_std);
    end

    % Perform the correction step of the EKF
    % initialization range for bearing only slam
    if ~isempty(data.timestep(t).sensor)
        init_depth = sup_data.doa_dist_max/2;
        [mu_ss, sigma_ss, observedLandmarks] = correction_step_2d(mu, sigma, data.timestep(t).sensor, observedLandmarks,init_depth,sup_data.obs_std);
        
%         mu(1:3) = mu_ss(1:3);
        mu(3+1:3+4*N) = mu_ss(3+1:3+4*N);
%         mu(3+4*N+1:end) = mu_ss(3+4*N+1:end);

        sigma(3+1:3+4*N,3+1:3+4*N) = sigma_ss(3+1:3+4*N,3+1:3+4*N);
        sigma(1:3,3+1:3+4*N) = sigma_ss(1:3,3+1:3+4*N);
        sigma(3+1:3+4*N,1:3) = sigma_ss(3+1:3+4*N,1:3);

%         sigma(1:3,1:3) = sigma_ss(1:3,1:3);

%         sigma(3+4*N+1:end,3+4*N+1:end) = sigma_ss(3+4*N+1:end,3+4*N+1:end);
%         sigma(1:3,3+4*N+1:end) = sigma_ss(1:3,3+4*N+1:end);
%         sigma(3+4*N+1:end,1:3) = sigma_ss(3+4*N+1:end,1:3);
        
        sigma(3+4*N+1:end,3+1:3+4*N) = sigma_ss(3+4*N+1:end,3+1:3+4*N);
        sigma(3+1:3+4*N,3+4*N+1:end) = sigma_ss(3+1:3+4*N,3+4*N+1:end);

%         mu = mu_ss;
%         sigma = sigma_ss;
    end
    
    % update for rblm
    if ~isempty(data.timestep(t).sensor_rblm)
        %init_depth = sup_data.doa_dist_max;
        [mu, sigma, observedLandmarks_rblm] = correction_step_rblm_2d(mu, sigma, data.timestep(t).sensor_rblm, observedLandmarks_rblm,sup_data.obs_rblm_azim_std,sup_data.obs_rblm_r_std,N,N_rblm);
    end

    robot_pose_hist = [robot_pose_hist,mu(1:3)];
    %Generate visualization plots of the current state of the filter
    plot_state_2d(mu, sigma, landmarks,landmarks_rblm, t, observedLandmarks,observedLandmarks_rblm, data.timestep(t).sensor,data.timestep(t).sensor_rblm, robot_pose_hist, robot_pose_gt_hist);
    disp('Current state vector:')
    disp('mu = '), disp(mu)
    
    pause(0.1);
end


%disp('Final system covariance matrix:'), disp(sigma)
% Display the final state estimate
%disp('Final robot pose:')
%disp('mu_robot = '), disp(mu(1:3)), disp('sigma_robot = '), disp(sigma(1:3,1:3))

lm_2d_xy_coord = zeros(N,2);
for n=1:N
  lm_2d_xy_coord(n,:) = [(1/mu(3+4*(n-1)+4))*cos(mu(3+4*(n-1)+3)) + mu(3+4*(n-1)+1), ...
      (1/mu(3+4*(n-1)+4))*sin(mu(3+4*(n-1)+3)) + mu(3+4*(n-1)+2)];
end

lm_2d_xy_coord_gt = zeros(N,2);
for n=1:N
  lm_2d_xy_coord_gt(n,:) = [landmarks(n).x landmarks(n).y];
end
  
% compute the estimation RMS error
%ss_pos_est = reshape(g2d.x(1:2*g2d.ss_num),2,g2d.ss_num)';
%ss_pos_gt = reshape(g2d.x_gt(1:2*g2d.ss_num),2,g2d.ss_num)';
ss_pos_est = lm_2d_xy_coord;
ss_pos_gt = lm_2d_xy_coord_gt;

ss_pos_e = sqrt((ss_pos_gt(:,1)-ss_pos_est(:,1)).^2 + (ss_pos_gt(:,2)-ss_pos_est(:,2)).^2);
ss_pos_e_rms = rms(ss_pos_e);

disp(['the RMS error of sound sources position: ' num2str(ss_pos_e_rms)]);

rmpath('tools/ekf_idp');