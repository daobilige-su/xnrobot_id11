% this function is monte carlo run of ssl with different slam methods

%% refresh
clear all;
close all;

rmpath('tools/lsslam');rmpath('tools/lsslam_idp');

%% default params

%%%
% params for creation of slam graph

% sound source DOA estimation noise 
ss_doa_std_azi = 10*(pi/180);
ss_doa_std_ele = 10*(pi/180);
ss_doa_std_azi_conservative_factor = 2;
ss_doa_std_ele_conservative_factor = 2;

% odometry noises
% odom_u_theta/ also yaw/pitch/roll
odom_u_theta_std = 0.001*(pi/180); %0.001 deg
odom_u_theta_std_conservative_factor = 2; 
% odom_u_dist_std
odom_u_dist_std = 0.001; % 1mm
odom_u_dist_std_conservative_factor = 2; 

% distance between each sound source
ss_dist = 4;

% dist between each robot pose
robot_pose_dist = 0.2;

% flag to decide if there is a loop close in trajectory
loop_closure_on = 1;
% flag to decide if plotting is needed
plot_on = 0;

% name of the output file
output_file_name = 'mic_array.mat';

%%%
% params

% % flag for using parallel computation
% parfor_on = 1;

% monte carlo run num
monte_carlo_run_num = 20;

% lsslam plot flag
lsslam_plot_on = 0;

% threshold of rms error for convergence detection
thr_rms = 1.0;

%% simulation for different odom noise

% result output file name
result_folder = '../data/';
result_file_name = 'lsslam_error_wrt_odom_noise.mat';

% back up default params
odom_u_theta_std_default = odom_u_theta_std;
odom_u_dist_std_default = odom_u_dist_std;
output_file_name_default = output_file_name;

range_step_num = 13;
odom_u_theta_std_range = 0.001*(pi/180)*(2.^(1-1:range_step_num-1)); %0.001 deg -- 4.096 deg, log scale every 0.2m
odom_u_dist_std_range = 0.001*(1:range_step_num); % 1mm -- 13mm every 0.2m

lsslam_2d_error = zeros(monte_carlo_run_num,range_step_num);
lsslam_2d_convergence = zeros(monte_carlo_run_num,range_step_num);
lsslam_2d_idp_error = zeros(monte_carlo_run_num,range_step_num);
lsslam_2d_idp_convergence = zeros(monte_carlo_run_num,range_step_num);

lsslam_3d_error = zeros(monte_carlo_run_num,range_step_num);
lsslam_3d_convergence = zeros(monte_carlo_run_num,range_step_num);
lsslam_3d_idp_error = zeros(monte_carlo_run_num,range_step_num);
lsslam_3d_idp_convergence = zeros(monte_carlo_run_num,range_step_num);

lsslam_2d_mean_rms              = zeros(1,range_step_num);
lsslam_2d_convergence_rate      = zeros(1,range_step_num);
lsslam_2d_idp_mean_rms          = zeros(1,range_step_num);
lsslam_2d_idp_convergence_rate  = zeros(1,range_step_num);
lsslam_3d_mean_rms              = zeros(1,range_step_num);
lsslam_3d_convergence_rate      = zeros(1,range_step_num);
lsslam_3d_idp_mean_rms          = zeros(1,range_step_num);
lsslam_3d_idp_convergence_rate  = zeros(1,range_step_num);

for k=1:range_step_num
    
    odom_u_theta_std = odom_u_theta_std_range(k);
    odom_u_dist_std = odom_u_dist_std_range(k);

    % monte carlo runs for each situation 
    parfor n = 1:monte_carlo_run_num
        %% graph creation
        
        output_file_name = ['mc_k_',num2str(k),'_n_',num2str(n),'.mat'];

        %run the main function for graph creation for lsslam
        create_graph_func(ss_doa_std_azi,ss_doa_std_ele,ss_doa_std_azi_conservative_factor,ss_doa_std_ele_conservative_factor,...
            odom_u_theta_std,odom_u_dist_std,odom_u_theta_std_conservative_factor,odom_u_dist_std_conservative_factor,...
            robot_pose_dist,ss_dist,loop_closure_on,plot_on,output_file_name);

        %% run lsslam

        % 2D slam cartesian
        [ss_pos_e_rms] = lsSLAM_2d_lm_func(output_file_name,lsslam_plot_on);
        lsslam_2d_error(n,k) = ss_pos_e_rms;
        if ss_pos_e_rms>thr_rms
            lsslam_2d_convergence(n,k) = 0;
        else
            lsslam_2d_convergence(n,k) = 1;
        end

        % 2D slam idp
        [ss_pos_e_rms] = lsSLAM_2d_lm_idp_func(output_file_name,lsslam_plot_on);
        lsslam_2d_idp_error(n,k) = ss_pos_e_rms;
        if ss_pos_e_rms>thr_rms
            lsslam_2d_idp_convergence(n,k) = 0;
        else
            lsslam_2d_idp_convergence(n,k) = 1;
        end

        % 3D slam cartesian
        [ss_pos_e_rms] = lsSLAM_3d_lm_func(output_file_name,lsslam_plot_on);
        lsslam_3d_error(n,k) = ss_pos_e_rms;
        if ss_pos_e_rms>thr_rms
            lsslam_3d_convergence(n,k) = 0;
        else
            lsslam_3d_convergence(n,k) = 1;
        end

        % 3D slam idp
        [ss_pos_e_rms] = lsSLAM_3d_lm_idp_func(output_file_name,lsslam_plot_on);
        lsslam_3d_idp_error(n,k) = ss_pos_e_rms;
        if ss_pos_e_rms>thr_rms
            lsslam_3d_idp_convergence(n,k) = 0;
        else
            lsslam_3d_idp_convergence(n,k) = 1;
        end


    end

    %% analyze result

    % lsslam_2d_mean_rms = 0;
    % lsslam_2d_convergence_rate = 0;

    lsslam_2d_mean_rms(k) = sum(lsslam_2d_error(:,k).*lsslam_2d_convergence(:,k),1)/sum(lsslam_2d_convergence(:,k),1);
    lsslam_2d_convergence_rate(k) = sum(lsslam_2d_convergence(:,k),1)/monte_carlo_run_num;

    lsslam_2d_idp_mean_rms(k) = sum(lsslam_2d_idp_error(:,k).*lsslam_2d_idp_convergence(:,k),1)/sum(lsslam_2d_idp_convergence(:,k),1);
    lsslam_2d_idp_convergence_rate(k) = sum(lsslam_2d_idp_convergence(:,k),1)/monte_carlo_run_num;

    lsslam_3d_mean_rms(k) = sum(lsslam_3d_error(:,k).*lsslam_3d_convergence(:,k),1)/sum(lsslam_3d_convergence(:,k),1);
    lsslam_3d_convergence_rate(k) = sum(lsslam_3d_convergence(:,k),1)/monte_carlo_run_num;

    lsslam_3d_idp_mean_rms(k) = sum(lsslam_3d_idp_error(:,k).*lsslam_3d_idp_convergence(:,k),1)/sum(lsslam_3d_idp_convergence(:,k),1);
    lsslam_3d_idp_convergence_rate(k) = sum(lsslam_3d_idp_convergence(:,k),1)/monte_carlo_run_num;
end

%% plot the result

result_fig_folder = '../plots/';

x_tick_labels{range_step_num} = struct;
for k=1:range_step_num
    x_tick_labels{k}=[num2str(odom_u_dist_std_range(k)),'m, ',num2str(odom_u_theta_std_range(k)*(180/pi)),'deg'];
end

% RMS error
figure,
hold on;
plot(lsslam_2d_mean_rms,'LineWidth',2);
plot(lsslam_2d_idp_mean_rms,'LineWidth',2);
plot(lsslam_3d_mean_rms,'LineStyle',':','LineWidth',2);
plot(lsslam_3d_idp_mean_rms,'LineStyle',':','LineWidth',2);
grid on;
xlabel('Odometry noise(unit:m/degree per 0.2m)');ylabel('Mean RMS error');
legend('2D euler','2D idp','3D euler','3D idp');
hold off;
set(gca,'Xtick',1:range_step_num)
set(gca,'XTickLabel',x_tick_labels);
set(gca,'XTickLabelRotation',45);

result_fig_file_name = 'lsslam_error_wrt_odom_noise_mean_rms';

saveas(gcf,[result_fig_folder result_fig_file_name '.fig']);
print(gcf,[result_fig_folder result_fig_file_name],'-dpng','-r300');
print(gcf,[result_fig_folder result_fig_file_name],'-deps','-r300');

% convergence rate
figure,
hold on;
plot(lsslam_2d_convergence_rate,'LineWidth',2);
plot(lsslam_2d_idp_convergence_rate,'LineWidth',2);
plot(lsslam_3d_convergence_rate,'LineStyle',':','LineWidth',2);
plot(lsslam_3d_idp_convergence_rate,'LineStyle',':','LineWidth',2);
grid on;
xlabel('Odometry noise(unit:m/degree per 0.2m)');ylabel('Convergence rate');
legend('2D euler','2D idp','3D euler','3D idp');
hold off;
set(gca,'Xtick',1:range_step_num)
set(gca,'XTickLabel',x_tick_labels);
set(gca,'XTickLabelRotation',45);

result_fig_file_name = 'lsslam_error_wrt_odom_noise_convergence_rate';

saveas(gcf,[result_fig_folder result_fig_file_name '.fig']);
print(gcf,[result_fig_folder result_fig_file_name],'-dpng','-r300');
print(gcf,[result_fig_folder result_fig_file_name],'-deps','-r300');

% recover default params
odom_u_theta_std = odom_u_theta_std_default;
odom_u_dist_std = odom_u_dist_std_default;
output_file_name = output_file_name_default;

% save result
save([result_folder result_file_name],'lsslam_2d_error','lsslam_2d_convergence',...
    'lsslam_2d_idp_error','lsslam_2d_idp_convergence','lsslam_3d_error','lsslam_3d_convergence',...
    'lsslam_3d_idp_error','lsslam_3d_idp_convergence',...
    'lsslam_2d_mean_rms','lsslam_2d_convergence_rate','lsslam_2d_idp_mean_rms','lsslam_2d_idp_convergence_rate',...
    'lsslam_3d_mean_rms','lsslam_3d_convergence_rate','lsslam_3d_idp_mean_rms','lsslam_3d_idp_convergence_rate',...
    'odom_u_theta_std_range','odom_u_dist_std_range','monte_carlo_run_num');

%%
