function [ss_pos_e_rms] = lsSLAM_2d_idp_func(graph_file_name)
% This file performs least square SLAM
% make sure the creat_graph.m has been run before running this file.


%% refresh
% more off;
% clear all;
% close all;

%% parameters

% the maximum number of iterations
numIterations = 50;

% maximum allowed dx
EPSILON = 1e-4;

% Error
err = 0;

% add path for including some tool functions
addpath('tools/lsslam_idp');

% load the graph into the variable "g"
folder = '../data/';
if nargin<1
    graph_file_name = 'mic_array.mat';
end
%load ../data/mic_array.mat
load([folder graph_file_name]);


%% start slSLAM

% plot the initial state of the graph
figure(1);
plot_graph_ini_2d(g2d, 0);
%plot_graph(g, i);
%filename = ['../plots/lsslam_' num2str(0) '.png'];
%print(filename, '-dpng');
%saveas(gcf, filename, 'png');
%filename = ['../plots/lsslam_' num2str(0) '.fig'];
%print(filename, '-dpng');
%saveas(gcf, filename, 'fig');

% compute the error for ground truth
gx = g2d.x;
g2d.x = g2d.x_gt;
disp(['ground truth error ' num2str(compute_global_error_2d(g2d))]);

% compute initial error for state vector
g2d.x = gx;
disp(['Initial error ' num2str(compute_global_error_2d(g2d))]);

% carry out the iterations
for i = 1:numIterations
  disp(['Performing iteration ', num2str(i)]);

  % solve the dx
  [dx,H] = linearize_and_solve_2d(g2d);

  % TODO: apply the solution to the state vector g.x
  g2d.x = g2d.x + dx;
  
  % make sure the right most mic ({M_x}th mic) is along positive x axis
  % compute the angle to rotate
  %rot_angle = asin(g2d.x((g2d.M_x-1)*4+2)/g2d.x((g2d.M_x-1)*4+1));
  % compute the homogeneous rotation matrix associated to angle
  %rot_matrix = [cos(rot_angle) sin(rot_angle) 0;
  %              -sin(rot_angle) cos(rot_angle) 0;
  %              0 0 1];

  % plot the current state of the graph
  figure(2);
  plot_graph_2d(g2d, i,H);
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
  err = compute_global_error_2d(g2d);

  % Print current error
  disp(['Current error ' num2str(err)]);

  % TODO: implement termination criterion as suggested on the sheet
  % 
  %if display_norm_dx_on>0
  %  disp(['norm(dx) = ' num2str(norm(dx))]);
  %end
  
  if (norm(dx)<EPSILON)
    break;
  end

end

% show final error
disp(['Final error ' num2str(err)]);

% compute the estimation RMS error
ss_pos_est = reshape(g2d.x(1:2*g2d.ss_num),2,g2d.ss_num)';
ss_pos_gt = reshape(g2d.x_gt(1:2*g2d.ss_num),2,g2d.ss_num)';

ss_pos_e = sqrt((ss_pos_gt(:,1)-ss_pos_est(:,1)).^2 + (ss_pos_gt(:,2)-ss_pos_est(:,2)).^2);
ss_pos_e_rms = rms(ss_pos_e);

disp(['the RMS error of sound sources position: ' num2str(ss_pos_e_rms)]);

addpath('tools/lsslam_idp');


end



