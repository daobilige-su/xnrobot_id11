function [ss_pos_e_rms] = lsSLAM_3d_lm_idp_func(graph_file_name,plot_on)
% This file performs least square SLAM
% make sure the creat_graph.m has been run before running this file.

% Problem: H is not recovered after adding lamda*diag(H). If recovered,
% when we plot uncertainty by P = inv(H), matrix reaches its singularity.
% Happens in both euler and cartisian 3D lsSLAM.

%% refresh
% more off;
% clear all;
% close all;

%% parameters

% the maximum number of iterations
numIterations = 50;

% maximum allowed dx
EPSILON = 1e-2;

% Error
err = 0;

% params in lm optimization
% v in the levenberg-marquard optimazation
lm_v = 2;
lm_lamda = 0;
lm_x_res = [];
lm_F_res = 0;
lm_tau = 1e-11;
lm_step_accepted = 0;

% plotting
plot_obs_on = 0;
plot_ini_obs_on = 1;

% add path for including some tool functions
addpath('tools/lsslam_idp');

% load the graph into the variable "g"
folder = '../data/';
if nargin<2
    plot_on = 0;
end
if nargin<1
    graph_file_name = 'mic_array.mat';
end
%load ../data/mic_array.mat
load([folder graph_file_name]);


%% start slSLAM
if plot_on
  % plot the initial state of the graph
  figure(1);
  plot_graph_ini_3d(g3d_idp, 0, plot_ini_obs_on);
end
%plot_graph(g, i);
%filename = ['../plots/lsslam_' num2str(0) '.png'];
%print(filename, '-dpng');
%saveas(gcf, filename, 'png');
%filename = ['../plots/lsslam_' num2str(0) '.fig'];
%print(filename, '-dpng');
%saveas(gcf, filename, 'fig');

% compute the error for ground truth
gx = g3d_idp.x;
g3d_idp.x = g3d_idp.x_gt;
disp(['ground truth error ' num2str(compute_global_error_3d(g3d_idp))]);

% compute initial error for state vector
g3d_idp.x = gx;
initial_error = compute_global_error_3d(g3d_idp);
disp(['Initial error ' num2str(initial_error)]);

lm_x_res = g3d_idp.x;
lm_F_res = initial_error;

% carry out the iterations
for i = 1:numIterations
  disp(['Performing iteration ', num2str(i)]);

  
  
  % solve the dx
  [dx,H,lm_rau,lm_F_new,lm_lamda] = linearize_and_solve_3d_lm(g3d_idp,lm_F_res,lm_lamda,lm_tau,i);

  if lm_rau>0
    lm_step_accepted = 1;
    dx = dx*1;
    lm_lamda = lm_lamda*max(1/3,1-(2*lm_rau-1)^3);
    lm_v=2;
  else
    lm_step_accepted = 0;
    dx = dx*0;
    disp('LM optimization step rejected: reverting lamda...')
    lm_lamda = lm_lamda*lm_v;
    lm_v = 2*lm_v;
  end
  
  if lm_step_accepted
    % TODO: apply the solution to the state vector g.x
    g3d_idp.x = g3d_idp.x + dx;

    % normalize quat
    g3d_idp.x = norm_quat_3d(g3d_idp.x,g3d_idp.ss_num);

    lm_x_res = g3d_idp.x;
  else
    g3d_idp.x = lm_x_res;
  end
  
  % make sure the right most mic ({M_x}th mic) is along positive x axis
  % compute the angle to rotate
  %rot_angle = asin(g2d.x((g2d.M_x-1)*4+2)/g2d.x((g2d.M_x-1)*4+1));
  % compute the homogeneous rotation matrix associated to angle
  %rot_matrix = [cos(rot_angle) sin(rot_angle) 0;
  %              -sin(rot_angle) cos(rot_angle) 0;
  %              0 0 1];

  if plot_on
    % plot the current state of the graph
    figure(2);
    plot_graph_3d(g3d_idp, i,H, plot_obs_on);
  end
  %filename = ['../plots/lsslam_' num2str(i) '.png'];
  %print(filename, '-dpng');
  %saveas(gcf, filename, 'png');
  %filename = ['../plots/lsslam_' num2str(i) '.fig'];
  %print(filename, '-dpng');
  %saveas(gcf, filename, 'fig');
  %{
  grid on;
  xlim([-0.5 g.mic_dis*(g.M_x-1)+(g.mic_dis/2)]);ylim([-0.5 g.mic_dis*(g.M_y-1)+(g.mic_dis/2)]);
  %}

  % compute current error
  if lm_step_accepted
    err = lm_F_new;
  else
    err = lm_F_res;
  end
  
  lm_F_res = err;

  % Print current error
  disp(['Current error ' num2str(err)]);

  % TODO: implement termination criterion as suggested on the sheet
  % 
  %if display_norm_dx_on>0
  %  disp(['norm(dx) = ' num2str(norm(dx))]);
  %end
  
  if lm_step_accepted
      if (norm(dx)<EPSILON)
        break;
      end
  end
end

% show final error
disp(['Final error ' num2str(err)]);

lm_3d_xy_coord = zeros(g3d_idp.ss_num,3);
for n=1:g3d_idp.ss_num
  lm_3d_xy_coord(n,:) = (g3d_idp.x(g3d_idp.ss_num*3+7*(g3d_idp.lm_ini_pose_correspondance_3d(n,2)-1)+1:g3d_idp.ss_num*3+7*(g3d_idp.lm_ini_pose_correspondance_3d(n,2)-1)+3) + ...
    (1/g3d_idp.x(3*(n-1)+3))*[cos(g3d_idp.x(3*(n-1)+2))*cos(g3d_idp.x(3*(n-1)+1));cos(g3d_idp.x(3*(n-1)+2))*sin(g3d_idp.x(3*(n-1)+1));sin(g3d_idp.x(3*(n-1)+2))])';
end

lm_3d_xy_coord_gt = zeros(g3d_idp.ss_num,3);
for n=1:g3d_idp.ss_num
  lm_3d_xy_coord_gt(n,:) = (g3d_idp.x_gt(g3d_idp.ss_num*3+7*(g3d_idp.lm_ini_pose_correspondance_3d(n,2)-1)+1:g3d_idp.ss_num*3+7*(g3d_idp.lm_ini_pose_correspondance_3d(n,2)-1)+3) + ...
    (1/g3d_idp.x_gt(3*(n-1)+3))*[cos(g3d_idp.x_gt(3*(n-1)+2))*cos(g3d_idp.x_gt(3*(n-1)+1));cos(g3d_idp.x_gt(3*(n-1)+2))*sin(g3d_idp.x_gt(3*(n-1)+1));sin(g3d_idp.x_gt(3*(n-1)+2))])';
end

% compute the estimation RMS error
%ss_pos_est = reshape(g3d.x(1:3*g3d.ss_num),3,g3d.ss_num)';
%ss_pos_gt = reshape(g3d.x_gt(1:3*g3d.ss_num),3,g3d.ss_num)';
ss_pos_est = lm_3d_xy_coord;
ss_pos_gt = lm_3d_xy_coord_gt;

ss_pos_e = sqrt((ss_pos_gt(:,1)-ss_pos_est(:,1)).^2 + (ss_pos_gt(:,2)-ss_pos_est(:,2)).^2 ...
    + (ss_pos_gt(:,3)-ss_pos_est(:,3)).^2);
ss_pos_e_rms = rms(ss_pos_e);

disp(['the RMS error of sound sources position: ' num2str(ss_pos_e_rms)]);

rmpath('tools/lsslam_idp');
end



