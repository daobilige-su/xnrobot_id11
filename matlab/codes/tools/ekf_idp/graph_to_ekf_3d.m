function graph_to_ekf_3d(file)
% this function create neccessary files from graph slam simulation data for
% EKF slam simulation.
%
% Attention: 
% (1) the simulation result will not be the same as graph slam result
% This is due to the fact that:
% the odometry noise in graph slam is in terms of (x,y,theta), while the
% odometry noise in EKF slam is in terms of (theta_1,dist_theta_2). The
% noise from graph slam is not properly reprojected to EKF motion terms,
% but simply obtained translation noise and rotation noise from graph slam
% odometry noise and used them in EKF odometry noise. 
% 
% However, in the code of Cyrill's EKF slam framework, the motion odometry 
% noise is in terms of (x,y,theta)... and there is no converstion from its 
% (theta_1,dist,theta_2) type of noise to (x,y,theta). So, at the end,
% graph slam and EKF can be the same for this 1st attention point.
%
% (2) the initial pose EKF is (0,0,0) and in Cyrill's code framework, they
% assumes that the robot doesn't observe any landmark in its initial pose.
% Here, we want to incorporate the situation that the robot might
% observe landmark in its initial pose. so our first odometry is u=(0,0,0).
% This means that robot is in its initial pose for twice.
%% load simulation data
% if nargin is none, then import the slam graph from lsslam simulation.

if nargin<1
    folder = '../../lsslam/data/';
    file = 'mic_array.mat';
    
    file = [folder file];
end
% load graph slam simulation data
load(file);

%% deal with the odometry first.

% translation noise
u_dist_std = [];
% rotation noise
u_theta_std = [];

odom_idx = 1;
% robot initial pose (x,y,z,qr,qx,qy,qz)
odom_pose_ini = [0.0;0.0;0.0;1.0;0.0;0.0;0.0];
% robot 1st odomery, it is the same as its initial pose in order to
% incorporate the situation where the robot see landmark in its initial
% pose
odom(1).u = [0.0;0.0;0.0;0.0;0.0;0.0]; % input in theta_1,dist,theta_2
odom(1).pose = odom_pose_ini; % robot pose from pure odometry data
odom(1).obs_idx = []; % observation index

% iterate for each edge in graph slam
for n=1:size(g3d.edges,2)
    % look for pose-pose constraints
    if g3d.edges(n).type == 'P'
        % if it is first p-p constraint, extract the translation and
        % rotation noise from it
        if isempty(u_dist_std)
            u_dist_std = sqrt((1/g3d.edges(n).information(1,1)));
            u_theta_std = sqrt((1/g3d.edges(n).information(4,4)));
        end
        % increase the odometry index
        odom_idx = odom_idx+1;
        % previous odometry pose
        odom_pose_pre = odom(odom_idx-1).pose;
        % control input u from graph slam
        u_graph = g3d.edges(n).measurement;
        % control input u for EKF slam
        u_ekf = u_graph;
        % current robot pose
        odom_pose_cur = pose3d_ypr_to_quat(pose_plus_pose_trans_ypr(pose3d_quat_to_ypr(odom_pose_pre),g3d.edges(n).measurement));
        %odom_pose_cur(3) = wrapToPi(odom_pose_cur(3));

        % store data in odom struct
        odom(odom_idx).u = u_ekf;
        odom(odom_idx).pose = odom_pose_cur;
        odom(odom_idx).obs_idx = [];
    elseif g3d.edges(n).type == 'L'
        continue;
    else
        disp('error: unknown edge type');
    end
end

%% deal with landmark now

% bearing only observation noise
obs_azim_std = [];
obs_elev_std = [];

obs_idx = 0;

% iterate for each edge in graph slam
for n=1:size(g3d.edges,2)
   if g3d.edges(n).type == 'P'
       continue;
   elseif g3d.edges(n).type == 'L'
       % if it is the 1st p-l constraint, extract observation nosie
       if isempty(obs_azim_std)
           obs_azim_std = sqrt((1/g3d.edges(n).information(1,1)));
           obs_elev_std = sqrt((1/g3d.edges(n).information(2,2)));
       end
       % increase the observation index
       obs_idx = obs_idx+1;
       % observed azimuth angle (bearing observation)
       obs_azim_elev = g3d.edges(n).measurement;
       % odometry index that corresponds to this landmark observation
       obs_odom_idx = (g3d.edges(n).toIdx-g3d.ss_num*3+6)/7;
       % observed landmark index
       obs_lm_idx = (g3d.edges(n).fromIdx+2)/3;
       
       % store data in obs struct
       obs(obs_idx).azim_elev = obs_azim_elev;
       obs(obs_idx).lm_idx = obs_lm_idx;
       
       % tell the corresponding odometry for this observation
       odom(obs_odom_idx).obs_idx = [odom(obs_odom_idx).obs_idx,obs_idx];
   else
       disp('error: unknown edge type');
   end
end

%% write files in data folder

% write the world.dat file
% ground true value for landmarks
lm_gt = reshape(g3d.x_gt(1:g3d.ss_num*3),3,g3d.ss_num)';
% ground true value and index for landmarks
lm_gt_with_idx = [(1:g3d.ss_num)',lm_gt];

% file path
world_dat_folder = '../data/';
world_dat_file_name = 'world_3d.dat';
% write the file
dlmwrite([world_dat_folder world_dat_file_name],lm_gt_with_idx,...
    'delimiter',' ','precision',12);

% write the sensor_data.dat file
% file path
sensor_data_dat_folder = '../data/';
sensor_data_dat_file = 'sensor_data_3d.dat';
sensor_data_dat_file_id = fopen([sensor_data_dat_folder,...
    sensor_data_dat_file],'w');

% iterate for each odometry
for n=1:size(odom,2)
    % wrtie odometry information
    odom_msg = ['ODOMETRY',' ',num2str(odom(n).u(1),'%0.12f'),' ',...
        num2str(odom(n).u(2),'%0.12f'),' ',num2str(odom(n).u(3),'%0.12f'),...
        ' ',num2str(odom(n).u(4),'%0.12f'),' ',num2str(odom(n).u(5),...
        '%0.12f'),' ',num2str(odom(n).u(6),'%0.12f'),'\n'];
    
    fprintf(sensor_data_dat_file_id,odom_msg);
    
    % if any landmark is observated when robot is in this pose, write it
    % down
    if ~isempty(odom(n).obs_idx)
        for k=1:size(odom(n).obs_idx,2)
            % landmark id
            sensor_id = obs(odom(n).obs_idx(k)).lm_idx;
            % landmark azimuth angle
            sensor_azim_elev = obs(odom(n).obs_idx(k)).azim_elev;
            
            % write down landmark information
            sensor_msg = ['SENSOR',' ',num2str(sensor_id),' ',num2str(...
                sensor_azim_elev(1),'%0.12f'),' ',num2str(...
                sensor_azim_elev(2),'%0.12f'),'\n'];
            fprintf(sensor_data_dat_file_id,sensor_msg);
        end
    end
end

fclose(sensor_data_dat_file_id);

% supplementary data
% odometry and observation nosie
sup_data.u_dist_std = u_dist_std;
sup_data.u_theta_std = u_theta_std;
sup_data.obs_azim_std = obs_azim_std;
sup_data.obs_elev_std = obs_elev_std;
% DOA max distance
sup_data.doa_dist_max = g3d.doa_dist_max;
% robot pose ground true
sup_data.robot_pose_gt = [[0,0,0,1.0,0,0,0];reshape(g3d.x_gt(g3d.ss_num*3+1:end),...
    7,[])'];

% save supplementary data in a mat file
save('../data/sup_data_3d.mat','sup_data');

end

