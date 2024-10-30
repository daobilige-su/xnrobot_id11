% This file performs least square SLAM
% make sure the creat_graph.m has been run before running this file.


%% refresh
more off;
clear all;
close all;

%% parameters

% gonna estimate clock drift?
est_drift_on = 1;
% gonna estimate starting time delay?
est_delay_on = 1;
% display starting time delay estimation error?
display_delay_error_on = 0;
% display norm(dx) for each iteration?
display_norm_dx_on = 0;

% the maximum number of iterations
numIterations = 50;

% maximum allowed dx
EPSILON = 1e-4;

% Error
err = 0;

% add path for including some tool functions
addpath('tools');

% load the graph into the variable "g"
load ../data/mic_array.mat

% if est_drift_on is not enabled, assign the ground truth values
if est_drift_on<1
    for n = 2:g.M
        g.x(4*(n-1)+4) = g.x_gt(4*(n-1)+4);
    end
end

% if est_delay_on is not enabled, assign the ground truth values
if est_delay_on<1
    for n = 2:g.M
        g.x(4*(n-1)+3) = g.x_gt(4*(n-1)+3);
    end
end


%% start slSLAM

% plot the initial state of the graph
plot_graph(g, 0);
plot_graph(g, i);
filename = ['../plots/lsslam_' num2str(0) '.png'];
%print(filename, '-dpng');
saveas(gcf, filename, 'png');
filename = ['../plots/lsslam_' num2str(0) '.fig'];
%print(filename, '-dpng');
saveas(gcf, filename, 'fig');

% compute the error for ground truth
gx = g.x;
g.x = g.x_gt;
disp(['ground truth error ' num2str(compute_global_error(g))]);

% compute initial error for state vector
g.x = gx;
disp(['Initial error ' num2str(compute_global_error(g))]);

% carry out the iterations
for i = 1:numIterations
  disp(['Performing iteration ', num2str(i)]);

  % solve the dx
  dx = linearize_and_solve(g,est_delay_on,est_drift_on);

  % TODO: apply the solution to the state vector g.x
  g.x = g.x + dx;
  
  % make sure the right most mic ({M_x}th mic) is along positive x axis
  % compute the angle to rotate
  rot_angle = asin(g.x((g.M_x-1)*4+2)/g.x((g.M_x-1)*4+1));
  % compute the homogeneous rotation matrix associated to angle
  rot_matrix = [cos(rot_angle) sin(rot_angle) 0;
                -sin(rot_angle) cos(rot_angle) 0;
                0 0 1];
  % rotate the mic positions
  for n=2:g.M
      hc = rot_matrix*[g.x(4*(n-1)+1);g.x(4*(n-1)+2);1];
      g.x(4*(n-1)+1) = hc(1);
      g.x(4*(n-1)+2) = hc(2);
  end
  % rotate the sound src positions
  for n=1:(size(g.x,1)-4*g.M)/2
      hc = rot_matrix*[g.x(2*(n-1)+1+4*g.M);g.x(2*(n-1)+2+4*g.M);1];
      g.x(2*(n-1)+1+4*g.M) = hc(1);
      g.x(2*(n-1)+2+4*g.M) = hc(2);
  end
      
  % display estimation error of mic delay if asked
  if display_delay_error_on > 0    
      x_3_error = (g.x(7:4:g.M*4-1) - g.x_gt(7:4:g.M*4-1));
      disp('estimation error of starting time delay: ');
      x_3_error'
  end

  % plot the current state of the graph
  plot_graph(g, i);
  filename = ['../plots/lsslam_' num2str(i) '.png'];
  %print(filename, '-dpng');
  saveas(gcf, filename, 'png');
  filename = ['../plots/lsslam_' num2str(i) '.fig'];
  %print(filename, '-dpng');
  saveas(gcf, filename, 'fig');
  %{
  grid on;
  xlim([-0.5 g.mic_dis*(g.M_x-1)+(g.mic_dis/2)]);ylim([-0.5 g.mic_dis*(g.M_y-1)+(g.mic_dis/2)]);
  %}

  % compute current error
  err = compute_global_error(g);

  % Print current error
  disp(['Current error ' num2str(err)]);

  % TODO: implement termination criterion as suggested on the sheet
  % 
  if display_norm_dx_on>0
    disp(['norm(dx) = ' num2str(norm(dx))]);
  end
  
  if (norm(dx)<EPSILON)
    break;
  end

end

% show final error
disp(['Final error ' num2str(err)]);

% gt
mic_gt = [g.x_gt(1:4:(4*(g.M-1)+1)) g.x_gt(2:4:(4*(g.M-1)+2)) g.x_gt(3:4:(4*(g.M-1)+3)) g.x_gt(4:4:(4*(g.M-1)+4))];
mic_est = [g.x(1:4:(4*(g.M-1)+1)) g.x(2:4:(4*(g.M-1)+2)) g.x(3:4:(4*(g.M-1)+3)) g.x(4:4:(4*(g.M-1)+4))];
src_gt = [g.x_gt((4*g.M+1):2:end) g.x_gt((4*g.M+2):2:end)];
src_est = [g.x((4*g.M+1):2:end) g.x((4*g.M+2):2:end)];

gcf;
hold on;
plot(mic_gt(:,1),mic_gt(:,2),'LineStyle','none','Marker','s','MarkerEdgeColor','g');
plot(src_gt(:,1),src_gt(:,2),'LineStyle','none','Marker','s','MarkerEdgeColor','c');
legend('est. mic. pos.','est. src. pos.','g.t. of mic. pos.','g.t. of src. pos.');

figure,
hold on;
plot(mic_est(:,3),'LineStyle','none','Marker','s','MarkerEdgeColor','b');
plot(mic_gt(:,3),'LineStyle','none','Marker','o','MarkerEdgeColor','r');
hold off;
grid on;
xlabel('No. of mic');ylabel('Starting time offset (s)');
legend('Estimation','Ground truth');

figure,
hold on;
plot(mic_est(:,4),'LineStyle','none','Marker','s','MarkerEdgeColor','b');
plot(mic_gt(:,4),'LineStyle','none','Marker','o','MarkerEdgeColor','r');
hold off;
grid on;
xlabel('No. of mic');ylabel('Clock difference (s)');
legend('Estimation','Ground truth');

mic_pos_e = sqrt((mic_gt(:,1)-mic_est(:,1)).^2 + (mic_gt(:,2)-mic_est(:,2)).^2);
mic_pos_e_rms = rms(mic_pos_e)





