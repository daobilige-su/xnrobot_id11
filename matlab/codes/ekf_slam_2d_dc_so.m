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
addpath('tools/ekf');

% Read world data, i.e. landmarks. The true landmark positions are not given to the robot
%landmarks = read_world_2d('../data/world_2d.dat');
% Read sensor readings, i.e. odometry and range-bearing sensor
%data = read_data_2d('../data/sensor_data_2d.dat');
% Read other supplementary information
load ../data/mic_array_rblm_2d.mat

INF = 1000;
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
mu = repmat([0.0], (2*N+2*N_rblm+3), 1);
mu(1:3) = [0.2;0.0;0];
robSigma = zeros(3);
robMapSigma = zeros(3,2*N+2*N_rblm);
mapSigma = INF*eye(2*N+2*N_rblm);
sigma = [[robSigma robMapSigma];[robMapSigma' mapSigma]];

mu_dc = repmat([0.0], (2*N+3), 1);
robMapSigma_dc = zeros(3,2*N);
mapSigma_dc = INF*eye(2*N);
sigma_dc = [[robSigma robMapSigma_dc];[robMapSigma_dc' mapSigma_dc]];

% Perform filter update for each odometry-observation pair read from the
% data file.
robot_pose_hist = [];
robot_pose_gt_hist = [];
robot_pose_gt = [];

doa_w_diff_min = 45*(pi/180);

ss_pool(N).data = [];

for t = 1:size(data.timestep, 2)
    
    robot_pose_gt = sup_data.robot_pose_gt(t+1,:)';
    robot_pose_gt_hist = [robot_pose_gt_hist,robot_pose_gt];

    % Perform the prediction step of the EKF
    [mu, sigma] = prediction_step_2d(mu, sigma, data.timestep(t).odometry, sup_data.u_dist_std, sup_data.u_theta_std);

    
    
    % update for rblm
    if ~isempty(data.timestep(t).sensor_rblm)
        %init_depth = sup_data.doa_dist_max;
        [mu, sigma, observedLandmarks_rblm] = correction_step_rblm_2d(mu, sigma, data.timestep(t).sensor_rblm, observedLandmarks_rblm,sup_data.obs_rblm_azim_std,sup_data.obs_rblm_r_std,N,N_rblm);
    end
    
    
    % update for ss landmark
    if ~isempty(data.timestep(t).sensor)
        mu_cur = mu(1:3);
        sigma_cur = sigma(1:3,1:3);
        [ss_pool,selected_ss_obs]=select_ss_obs(mu_cur,sigma_cur,data.timestep(t).sensor,ss_pool,doa_w_diff_min);
    else
        selected_ss_obs = [];
    end
    
    % Perform the correction step of the EKF
    % initialization range for bearing only slam
%     mu_dc(1:3) = mu(1:3);
%     sigma_dc(1:3,1:3) = sigma(1:3,1:3);
    
    if ~isempty(selected_ss_obs)
        for k=1:size(selected_ss_obs,2)
            
            mu_dc(1:3) = selected_ss_obs(k).mu;
            sigma_dc(1:3,1:3) = selected_ss_obs(k).sigma;
            
            init_depth = sup_data.doa_dist_max;
            [mu_dc, sigma_dc, observedLandmarks] = correction_step_2d_dc(mu_dc, sigma_dc, selected_ss_obs(k).z, observedLandmarks,init_depth,sup_data.obs_std);

        end
    end
    
    
    
    robot_pose_hist = [robot_pose_hist,mu(1:3)];
    %Generate visualization plots of the current state of the filter
    plot_state_2d_dc(mu, sigma, landmarks,landmarks_rblm, t, observedLandmarks,observedLandmarks_rblm, data.timestep(t).sensor,data.timestep(t).sensor_rblm, robot_pose_hist, robot_pose_gt_hist,mu_dc,sigma_dc);
    %disp('Current state vector:')
    %disp('mu = '), disp(mu)
    
    pause(0.1);
end

%disp('Final system covariance matrix:'), disp(sigma)
% Display the final state estimate
%disp('Final robot pose:')
%disp('mu_robot = '), disp(mu(1:3)), disp('sigma_robot = '), disp(sigma(1:3,1:3))
