% This file create graph structure "g" for lsSLAM.m

% existing problems:
% doa_ob_w and doa_ob_w_2d are computed by doa_ob_l turns into doa_ob_w by
% robot pose gt instead of observed robot pose. doa_ob_w and doa_ob_w_2d
% are used for computing initial idp. Since robot pose has very little
% noise, it might be fine.


%% refresh
clear all;
close all;

%% Parameters

% add path for including some tool functions
addpath('tools');

% rect trajectory length per x,y
traj_len_x = 12;
traj_len_y = 12;

% distance from sound source to mic array trajectory
ss_dist_from_traj = 1;

% distance between each sound source
ss_dist = 4;

% number of sound sources
ss_num_x = (traj_len_x/ss_dist-1);
ss_num_y = (traj_len_y/ss_dist-1);
ss_num = 2*(ss_num_x + ss_num_y);

% sound source position noise in x/y direction
ss_pos_noise_std_xy = 0.1;

% sound source position noise in z direction
ss_pos_noise_std_z = 0.5;

% sound source DOA estimation noise 
ss_doa_std_azi = 10*(pi/180);
ss_doa_std_ele = 10*(pi/180);
ss_doa_std_azi_conservative_factor = 2;
ss_doa_std_ele_conservative_factor = 2;

% dist between each robot pose
robot_pose_dist = 0.2;

% num of robot pose
robot_pose_num_x = traj_len_x/robot_pose_dist;
robot_pose_num_y = traj_len_y/robot_pose_dist;
robot_pose_num = (robot_pose_num_x+robot_pose_num_y)*2 + robot_pose_num_x;

% max dist that doa algorithm can detect
doa_dist_max = 3;
% doa distance for initialization of sound source.
ss_pos_ini_doa_dist = 1.5;

% odometry noises
% odom_u_theta/ also yaw/pitch/roll
odom_u_theta_std = 0.001*(pi/180); %0.001 deg
odom_u_theta_std_conservative_factor = 2; 
% odom_u_dist_std
odom_u_dist_std = 0.001; % 1mm
odom_u_dist_std_conservative_factor = 2; 

% fig numbers

fig_2d_gt_num = 1;% 2D gt
fig_2d_ob_num = 3;% 2D ob
fig_3d_gt_num = 2;% 3D gt
fig_3d_ob_num = 4;% 3D ob

%% sound source ground truth

% ground truth of sound source positions
ss_pos = zeros(ss_num,3);

% x,y,z pos of 1st side of rect
ss_pos(1:ss_num_x,1) = (ss_dist:ss_dist:ss_num_x*ss_dist)'+...
    randn(ss_num_x,1)*ss_pos_noise_std_xy;
ss_pos(1:ss_num_x,2) = zeros(ss_num_x,1)+randn(ss_num_x,1)*...
    ss_pos_noise_std_xy + ss_dist_from_traj*sign(rand(ss_num_x,1)-0.5);
ss_pos(1:ss_num_x,3) = zeros(ss_num_x,1)+randn(ss_num_x,1)*...
    ss_pos_noise_std_z;

% x,y,z pos of 2nd side of rect
ss_pos(ss_num_x+1:ss_num_x+ss_num_y,1) = ones(ss_num_y,1)*traj_len_x+...
    randn(ss_num_y,1)*ss_pos_noise_std_xy + ss_dist_from_traj*sign(rand(...
    ss_num_y,1)-0.5);
ss_pos(ss_num_x+1:ss_num_x+ss_num_y,2) = (ss_dist:ss_dist:ss_num_y*...
    ss_dist)'+randn(ss_num_y,1)*ss_pos_noise_std_xy;
ss_pos(ss_num_x+1:ss_num_x+ss_num_y,3) = zeros(ss_num_y,1)+randn(...
    ss_num_y,1)*ss_pos_noise_std_z;

% x,y,z pos of 3rd side of rect
ss_pos(ss_num_x+ss_num_y+1:ss_num_x+ss_num_y+ss_num_x,1) = (ss_dist:...
    ss_dist:ss_num_x*ss_dist)'+randn(ss_num_x,1)*ss_pos_noise_std_xy;
ss_pos(ss_num_x+ss_num_y+1:ss_num_x+ss_num_y+ss_num_x,2) = ones(...
    ss_num_x,1)*traj_len_y+randn(ss_num_x,1)*ss_pos_noise_std_xy + ...
    ss_dist_from_traj*sign(rand(ss_num_x,1)-0.5);
ss_pos(ss_num_x+ss_num_y+1:ss_num_x+ss_num_y+ss_num_x,3) = zeros(...
    ss_num_x,1)+randn(ss_num_x,1)*ss_pos_noise_std_z;

% x,y,z pos of 4th side of rect
ss_pos(2*ss_num_x+ss_num_y+1:2*ss_num_x+ss_num_y+ss_num_y,1) = zeros(...
    ss_num_y,1)+randn(ss_num_y,1)*ss_pos_noise_std_xy+ ss_dist_from_traj...
    *sign(rand(ss_num_y,1)-0.5);
ss_pos(2*ss_num_x+ss_num_y+1:2*ss_num_x+ss_num_y+ss_num_y,2) = (ss_dist...
    :ss_dist:ss_num_y*ss_dist)'+randn(ss_num_y,1)*ss_pos_noise_std_xy;
ss_pos(2*ss_num_x+ss_num_y+1:2*ss_num_x+ss_num_y+ss_num_y,3) = zeros(...
    ss_num_y,1)+randn(ss_num_y,1)*ss_pos_noise_std_z;



% plot for debugging
%figure,
%plot(mic_pos(:,1),mic_pos(:,2),'LineStyle','none','Marker','s');

%% construction of robot odometry (ground truth)

% ini of robot poses (x,y,z,yaw,pitch,roll)
robot_pose = zeros(robot_pose_num,6);

% x,y,z pos of 1st side of rect
robot_pose(1:robot_pose_num_x,1) = (robot_pose_dist:robot_pose_dist:robot_pose_num_x*robot_pose_dist)';
robot_pose(1:robot_pose_num_x,2) = zeros(robot_pose_num_x,1);
robot_pose(1:robot_pose_num_x,3) = zeros(robot_pose_num_x,1);
robot_pose(1:robot_pose_num_x,4) = zeros(robot_pose_num_x,1);

% x,y,z pos of 2nd side of rect
robot_pose(robot_pose_num_x+1:robot_pose_num_x+robot_pose_num_y,1) = ones(robot_pose_num_y,1)*traj_len_x;
robot_pose(robot_pose_num_x+1:robot_pose_num_x+robot_pose_num_y,2) = (robot_pose_dist:robot_pose_dist:robot_pose_num_y*robot_pose_dist)';
robot_pose(robot_pose_num_x+1:robot_pose_num_x+robot_pose_num_y,3) = zeros(robot_pose_num_y,1);
robot_pose(robot_pose_num_x+1:robot_pose_num_x+robot_pose_num_y,4) = ones(robot_pose_num_y,1)*(pi/2);

% x,y,z pos of 3rd side of rect
robot_pose(robot_pose_num_x+robot_pose_num_y+1:robot_pose_num_x+robot_pose_num_y+robot_pose_num_x,1) = ((robot_pose_num_x-1)*robot_pose_dist:-robot_pose_dist:0)';
robot_pose(robot_pose_num_x+robot_pose_num_y+1:robot_pose_num_x+robot_pose_num_y+robot_pose_num_x,2) = ones(robot_pose_num_x,1)*traj_len_y;
robot_pose(robot_pose_num_x+robot_pose_num_y+1:robot_pose_num_x+robot_pose_num_y+robot_pose_num_x,3) = zeros(robot_pose_num_x,1);
robot_pose(robot_pose_num_x+robot_pose_num_y+1:robot_pose_num_x+robot_pose_num_y+robot_pose_num_x,4) = ones(robot_pose_num_x,1)*(pi);

% x,y,z pos of 4th side of rect
robot_pose(2*robot_pose_num_x+robot_pose_num_y+1:2*robot_pose_num_x+robot_pose_num_y+robot_pose_num_y,1) = zeros(robot_pose_num_y,1);
robot_pose(2*robot_pose_num_x+robot_pose_num_y+1:2*robot_pose_num_x+robot_pose_num_y+robot_pose_num_y,2) = ((robot_pose_num_y-1)*robot_pose_dist:-robot_pose_dist:0)';
robot_pose(2*robot_pose_num_x+robot_pose_num_y+1:2*robot_pose_num_x+robot_pose_num_y+robot_pose_num_y,3) = zeros(robot_pose_num_y,1);
robot_pose(2*robot_pose_num_x+robot_pose_num_y+1:2*robot_pose_num_x+robot_pose_num_y+robot_pose_num_y,4) = ones(robot_pose_num_y,1)*(-pi/2);

% x,y,z pos of 5th side of rect
robot_pose(2*robot_pose_num_x+2*robot_pose_num_y+1:2*robot_pose_num_x+2*robot_pose_num_y+robot_pose_num_x,1) = (robot_pose_dist:robot_pose_dist:robot_pose_num_x*robot_pose_dist)';
robot_pose(2*robot_pose_num_x+2*robot_pose_num_y+1:2*robot_pose_num_x+2*robot_pose_num_y+robot_pose_num_x,2) = zeros(robot_pose_num_x,1);
robot_pose(2*robot_pose_num_x+2*robot_pose_num_y+1:2*robot_pose_num_x+2*robot_pose_num_y+robot_pose_num_x,3) = zeros(robot_pose_num_y,1);
robot_pose(2*robot_pose_num_x+2*robot_pose_num_y+1:2*robot_pose_num_x+2*robot_pose_num_y+robot_pose_num_x,4) = zeros(robot_pose_num_y,1);


% plot the microphone and sound src ground truth
figure(fig_2d_gt_num);
hold on;
plot(ss_pos(:,1),ss_pos(:,2),'LineStyle','none','Marker','s',...
    'MarkerEdgeColor','r');
plot(robot_pose(:,1),robot_pose(:,2),'LineStyle','none','Marker','x',...
    'MarkerEdgeColor','b');
xlabel('x position (m)');ylabel('y position (m)');
legend('sound src','robot pose');
grid on;axis equal;
hold off;

fig_h = figure(fig_3d_gt_num);
set(fig_h, 'Position', [100, 100, 1000, 500]);
hold on;
plot3(ss_pos(:,1),ss_pos(:,2),ss_pos(:,3),'LineStyle','none','Marker','s',...
    'MarkerEdgeColor','r');
plot3(robot_pose(:,1),robot_pose(:,2),robot_pose(:,3),'LineStyle','none','Marker','x',...
    'MarkerEdgeColor','b');
xlabel('x position (m)');ylabel('y position (m)');zlabel('z position (m)');
legend('sound src','robot pose');
grid on;axis equal;view(45,10);
hold off;

%% compute the observation in azimuth and elevation angle.

% 2D case:
% project the robot pose from 3D to 2D case
robot_pose_2d = robot_pose(:,[1,2,4]);% x,y,yaw
% project the sound source position from 3D to 2D case
ss_pos_2d = ss_pos(:,[1,2]);

doa_gt_w_2d =[];% doa in world coord
doa_gt_l_2d =[];% doa in robot local coord
%doa_ob_w_2d =[];% observed (noisy) doa in world coord
doa_ob_l_2d =[];% observed (noisy) doa in local coord

for p = 1:size(robot_pose,1)
    for s = 1:size(ss_pos,1)
        if sqrt((robot_pose_2d(p,1)-ss_pos_2d(s,1))^2 + (robot_pose_2d(p,2)-ss_pos_2d(s,2))^2)<doa_dist_max
            azi_w_2d = atan2(ss_pos_2d(s,2)-robot_pose_2d(p,2),ss_pos_2d(s,1)-robot_pose_2d(p,1));
            
            azi_l_2d = wrapToPi(azi_w_2d-robot_pose_2d(p,3));
            
            azi_ob_l_2d = wrapToPi(azi_l_2d+randn*ss_doa_std_azi);
            
            %azi_ob_w_2d = wrapToPi(azi_ob_l_2d+robot_pose_2d(p,3));
            
            % store the data
            doa_gt_w_2d = [doa_gt_w_2d; [azi_w_2d,p,s]];
            doa_gt_l_2d = [doa_gt_l_2d; [azi_l_2d,p,s]];
            %doa_ob_w_2d = [doa_ob_w_2d; [azi_ob_w_2d,p,s]];
            doa_ob_l_2d = [doa_ob_l_2d; [azi_ob_l_2d,p,s]];
        end
    end
end




% 3D case:
% doa estimation ground truth
doa_gt_w =[];% doa in world coord
doa_gt_l =[];% doa in robot local coord
doa_ob_l_direc_point =[];% observed (noisy) doa in world coord
doa_ob_l =[];% observed (noisy) doa in local coord

for p = 1:size(robot_pose,1)
    for s = 1:size(ss_pos,1)
        if sqrt((robot_pose(p,1)-ss_pos(s,1))^2 + (robot_pose(p,2)-ss_pos(s,2))^2 + (robot_pose(p,3)-ss_pos(s,3))^2)<doa_dist_max
            
            % gt of DOA estimation azimuth angle in world coordinate
            azi_w = atan((ss_pos(s,2)-robot_pose(p,2))/(ss_pos(s,1)-robot_pose(p,1)));
            if (ss_pos(s,1)-robot_pose(p,1))<0
                azi_w = wrapToPi(azi_w+pi);
            end
            % observed DOA estimation azimuth angle in global coord
            % azi_ob_w = wrapToPi(azi_w+randn*ss_doa_std_azi);
            
            % azimuth gt in robot coordinate
            ss_pos_local_coord = global_point_in_local_coord_tans_ypr(ss_pos(s,:),robot_pose(p,:));
            % gt of DOA estimation azimuth angle in robot coordinate
            azi_l = atan(ss_pos_local_coord(2)/ss_pos_local_coord(1));
            if (ss_pos_local_coord(1))<0
                azi_l = wrapToPi(azi_l+pi);
            end
            % observed DOA estimation azimuth angle in local coord
            azi_ob_l = wrapToPi(azi_l+randn*ss_doa_std_azi);
            %azi_ob_w = 
            
            
            
            % gt of DOA estimation elevation angle in world coord
            ele_w = atan((ss_pos(s,3)-robot_pose(p,3))/sqrt((ss_pos(s,1)-robot_pose(p,1))^2+(ss_pos(s,2)-robot_pose(p,2))^2));
            % observed DOA estimation elevation angle in world coord
            %ele_ob_w = ele_w + randn*ss_doa_std_ele;
            %if ele_ob_w>(pi/2)
            %    ele_ob_w = pi/2;
            %elseif ele_ob_w<(-pi/2)
            %    ele_ob_w = -pi/2;
            %end
            
            % gt of DOA estimation elevation angle in local coord
            ele_l = atan(ss_pos_local_coord(3)/sqrt(ss_pos_local_coord(1)^2+ss_pos_local_coord(2)^2));
            % observed DOA estimation elevation angle in local coord
            ele_ob_l = ele_l + randn*ss_doa_std_ele;
            if ele_ob_l>(pi/2)
                ele_ob_l = pi/2;
            elseif ele_ob_l<(-pi/2)
                ele_ob_l = -pi/2;
            end
            
            % get the observation in world coordinate
            direc_point_ob_l = [cos(ele_ob_l)*cos(azi_ob_l);cos(ele_ob_l)*sin(azi_ob_l);sin(ele_ob_l)];
            
            
            
            % store the data
            doa_gt_w = [doa_gt_w; [azi_w,ele_w,p,s]];
            doa_ob_l_direc_point = [doa_ob_l_direc_point; [direc_point_ob_l',p,s]];
            doa_gt_l = [doa_gt_l; [azi_l,ele_l,p,s]];
            doa_ob_l = [doa_ob_l; [azi_ob_l,ele_ob_l,p,s]];
        end
    end
end

% plot 2D gt of robot pose, landmark, and observation
figure(fig_2d_gt_num);
hold on;
for n=1:size(doa_gt_w_2d,1)
    plot([robot_pose_2d(doa_gt_w_2d(n,2),1),ss_pos_2d(doa_gt_w_2d(n,3),1)],[robot_pose_2d(doa_gt_w_2d(n,2),2),ss_pos_2d(doa_gt_w_2d(n,3),2)],'Color','m');
end
hold off;

% plot 3D gt of robot pose, landmark, and observation
figure(fig_3d_gt_num);
hold on;
for n=1:size(doa_gt_w,1)
    plot3([robot_pose(doa_gt_w(n,3),1),ss_pos(doa_gt_w(n,4),1)],[robot_pose(doa_gt_w(n,3),2),ss_pos(doa_gt_w(n,4),2)],[robot_pose(doa_gt_w(n,3),3),ss_pos(doa_gt_w(n,4),3)],'Color','m');
end
grid on;axis equal;view(45,30);
hold off;

%% compute the odom noisy observation

% 2D case:
robot_pose_u_2d = zeros(size(robot_pose,1),3);
robot_pose_u_ob_2d = zeros(size(robot_pose,1),3);
robot_pose_ob_2d = zeros(size(robot_pose,1),3);

% computing the relative moment between two robot pose and its noisy
% observation.
robot_pose_ob_2d(1,:) = robot_pose_2d(1,:);
for n=2:size(robot_pose)
    robot_pose_u_2d(n,:) =pose_minus_pose_2d(robot_pose_2d(n,:),robot_pose_2d(n-1,:));
    robot_pose_u_ob_2d(n,:) = robot_pose_u_2d(n,:) + [randn(1,2)*odom_u_dist_std,randn*odom_u_theta_std];
    
    robot_pose_ob_2d(n,:) = pose_plus_pose_2d(robot_pose_ob_2d(n-1,:),robot_pose_u_ob_2d(n,:));
end

% observed doa in world coordinate from estimated robot pose (observed from noisy odom)
doa_ob_w_2d = zeros(size(doa_ob_l_2d));
doa_ob_w_2d(:,2:3) = doa_ob_l_2d(:,2:3);
for n=1:size(doa_ob_l_2d,1)
    doa_ob_w_2d(n,1) = wrapToPi(doa_ob_l_2d(n,1)+robot_pose_ob_2d(doa_ob_l_2d(n,2),3));
end

% plot noisy odom and landmarks
plot_initial_odom_and_landmark_2d(robot_pose_ob_2d,doa_ob_l_2d,doa_dist_max,fig_2d_ob_num);


% 3D case:
robot_pose_u_3d = zeros(size(robot_pose,1),6);
robot_pose_u_ob_3d = zeros(size(robot_pose,1),6);
robot_pose_ob_3d = zeros(size(robot_pose,1),6);

% computing the relative moment between two robot pose and its noisy
% observation.
robot_pose_ob_3d(1,:) = robot_pose(1,:);
for n=2:size(robot_pose)
    robot_pose_u_3d(n,:) = pose_minus_pose_trans_ypr(robot_pose(n,:),robot_pose(n-1,:))';
    robot_pose_u_ob_3d(n,:) = robot_pose_u_3d(n,:) + [randn(1,3)*odom_u_dist_std,randn(1,3)*odom_u_theta_std];
    
    robot_pose_ob_3d(n,:) = pose_plus_pose_trans_ypr(robot_pose_ob_3d(n-1,:),robot_pose_u_ob_3d(n,:));
end

% observed doa in world coordinate from estimated robot pose (observed from noisy odom)
doa_ob_w = zeros(size(doa_ob_l_direc_point,1),4);
doa_ob_w(:,3:4) = doa_ob_l_direc_point(:,4:5);
for n=1:size(doa_ob_l_direc_point,1)
    current_robot_pose_ob_3d_without_trans = robot_pose_ob_3d(doa_ob_l_direc_point(n,4),:);
    current_robot_pose_ob_3d_without_trans(1:3) = zeros(1,3);
    M = transform_matrix_from_trans_ypr(current_robot_pose_ob_3d_without_trans);
    local_point_without_trans_hom = M*[doa_ob_l_direc_point(n,1:3)';1];
    
    doa_ob_w(n,1) = atan2(local_point_without_trans_hom(2),local_point_without_trans_hom(1));
    doa_ob_w(n,2) = atan2(local_point_without_trans_hom(3),sqrt(local_point_without_trans_hom(1)^2+local_point_without_trans_hom(2)^2));
end

% plot noisy odom and landmarks
plot_initial_odom_and_landmark_3d(robot_pose_ob_3d,doa_ob_l,doa_dist_max,fig_3d_ob_num);


%% ground truth and initialization of state vector

% in 2D case,

% in this case, the state vector is sound src 1 (x, y), sound src 2 (x,y)
% ... robot pose 1 (x,y,theta), robot pose 2 (x,y,theta)...
%c2d_x_gt = [reshape(ss_pos(:,[1,2])',size(ss_pos,1)*2,1);reshape(robot_pose_2d',size(robot_pose_2d,1)*3,1)];
c2d_x_gt = [zeros(ss_num*2,1);reshape(robot_pose_2d',size(robot_pose_2d,1)*3,1)];

g2d.x_gt = c2d_x_gt;

% initialization of sound source positions
x_ini = zeros(ss_num*2+size(robot_pose_2d,1)*3,1);
% fill the noisy odom data
x_ini(ss_num*2+1:end) = reshape(robot_pose_ob_2d',size(robot_pose_ob_2d,1)*3,1);
% initialization of sound source positions
%ss_ini_index = 1;
ss_ini_observed = [];

lm_ini_pose_correspondance_2d = zeros(ss_num,2);
lm_ini_pose_correspondance_2d(:,1) = 1:ss_num;
for n=1:size(doa_ob_l_2d,1)
    if size(ss_ini_observed,2)==ss_num
        break;
    end
    if sum(doa_ob_l_2d(n,3)==ss_ini_observed)<1
        
        % initialization of sound source position upon 1st observation.
        % local coordinates:
        % ss_ini_pos_local_coord = ss_pos_ini_doa_dist*[cos(doa_ob_l_2d(n,1));sin(doa_ob_l_2d(n,1))];
        % change them into global coordinate:
        %ss_ini_pos_global_coord_hom = transform_matrix_from_pose_2d(robot_pose_ob_2d(doa_ob_l_2d(n,2),:))*[ss_ini_pos_local_coord;1];
        %ss_ini_pos_global_coord = ss_ini_pos_global_coord_hom(1:2);
        % fill the initial location
        %x_ini(2*(doa_ob_l_2d(n,3)-1)+1:2*(doa_ob_l_2d(n,3)-1)+2) = ss_ini_pos_global_coord;
        x_ini(2*(doa_ob_l_2d(n,3)-1)+1:2*(doa_ob_l_2d(n,3)-1)+2) = [doa_ob_w_2d(n,1);1/ss_pos_ini_doa_dist];
        
        ss_pos_robot_pose_dist_gt = sqrt(sum((robot_pose_ob_2d(doa_ob_w_2d(n,2),1:2)-ss_pos_2d(doa_ob_w_2d(n,3),1:2)).^2));
        g2d.x_gt(2*(doa_ob_l_2d(n,3)-1)+1:2*(doa_ob_l_2d(n,3)-1)+2) = [doa_gt_w_2d(n,1);1/ss_pos_robot_pose_dist_gt];
        
        lm_ini_pose_correspondance_2d(doa_ob_l_2d(n,3),2) = doa_ob_l_2d(n,2);
        
        %ss_ini_index = ss_ini_index+1;
        ss_ini_observed = [ss_ini_observed,doa_ob_l_2d(n,3)];
    end
    
end
% save data into graph
g2d.x = x_ini;
g2d.lm_ini_pose_correspondance_2d = lm_ini_pose_correspondance_2d;

lm_ini_2d_xy_coord = zeros(ss_num,2);
for n=1:ss_num
    lm_ini_2d_xy_coord(n,:) = (x_ini(ss_num*2+3*(lm_ini_pose_correspondance_2d(n,2)-1)+1:ss_num*2+3*(lm_ini_pose_correspondance_2d(n,2)-1)+2) + ...
        (1/x_ini(2*(n-1)+2))*[cos(x_ini(2*(n-1)+1));sin(x_ini(2*(n-1)+1))])';
end
% plot the initialization of sound sources
figure(fig_2d_ob_num);
hold on;
plot(lm_ini_2d_xy_coord(:,1),lm_ini_2d_xy_coord(:,2),'LineStyle','none','Marker','o',...
    'MarkerSize',10,'MarkerEdgeColor','k','MarkerFaceColor','y');
hold off;



% in 3D case,

% in this case, the state vector is sound src 1 (x, y,z), sound src 2 (x,y,z)
% ... robot pose 1 (x,y,z,yaw,pitch,roll), robot pose 2 (x,y,z,yaw,pitch,roll)...
% Note, in this case, yaw pitch roll angle is in sequence, just like that in
% airplane. first rotate by yaw(z axis), then pitch (y axis) and roll (x
% axis). So, this sequence is also called 'ZYX' sequence in matlab doc. All
% rotation is relative to local reference frame. This means airplane first
% rotate along its yaw angle, then in that new frame, change its pitch
% angle and so on...
% this can be tested by running a=angle2quat([0,pi/2],[pi/2,pi/2],[0,pi/2])
% here you can find that turning all 90 deg in a sequence yaw pitch roll, is
% actually the same as turning 90 deg in pitch angle only :D
c3d_x_gt_ypr = [zeros(ss_num*3,1);reshape(robot_pose',size(robot_pose,1)*6,1)];

% from Yaw/Pitch/roll rotation to quaternion, robot_pose_quat is Nx4 matrix
% [qr,qx,qy,qz; ...].
robot_pose_quat = [[robot_pose(:,1),robot_pose(:,2),robot_pose(:,3)],angle2quat(robot_pose(:,4),robot_pose(:,5),robot_pose(:,6))]; 
% state vector with coordinate transform in quaternion
c3d_x_gt_quat = [zeros(ss_num*3,1);reshape(robot_pose_quat',size(robot_pose_quat,1)*7,1)];

% change between two options. quaternion option seems to be more popular in
% SLAM since it can avoid directional singularity in YPR style expression.
% In YPR expression, the pitch angle has singularity around its -pi/2 and
% +pi/2 

% g3d.x_gt = c3d_x_gt_ypr;
g3d.x_gt = c3d_x_gt_quat;


% initialization of sound source positions
x_ini = zeros(ss_num*3+size(robot_pose,1)*7,1);
% fill the noisy odom data
robot_pose_ob_quat = [[robot_pose_ob_3d(:,1),robot_pose_ob_3d(:,2),robot_pose_ob_3d(:,3)],...
    angle2quat(robot_pose_ob_3d(:,4),robot_pose_ob_3d(:,5),robot_pose_ob_3d(:,6))];
x_ini(ss_num*3+1:end) = reshape(robot_pose_ob_quat',size(robot_pose_ob_quat,1)*7,1);
% initialization of sound source positions
%ss_ini_index = 1;
ss_ini_observed = [];

lm_ini_pose_correspondance_3d = zeros(ss_num,2);
lm_ini_pose_correspondance_3d(:,1) = 1:ss_num;
for n=1:size(doa_ob_l,1)
    if size(ss_ini_observed,2)==ss_num
        break;
    end
    if sum(doa_ob_l(n,4)==ss_ini_observed)<1
        
        % initialization of sound source position upon 1st observation.
        % local coordinates:
        %ss_ini_pos_local_coord = ss_pos_ini_doa_dist*[cos(doa_ob_l(n,2))*cos(doa_ob_l(n,1));...
        %    cos(doa_ob_l(n,2))*sin(doa_ob_l(n,1));sin(doa_ob_l(n,2))];
        % change them into global coordinate:
        %ss_ini_pos_global_coord_hom = transform_matrix_from_trans_ypr(robot_pose_ob_3d(doa_ob_l(n,3),:))*[ss_ini_pos_local_coord;1];
        %ss_ini_pos_global_coord = ss_ini_pos_global_coord_hom(1:3);
        % fill the initial location
        %x_ini(3*(doa_ob_l(n,4)-1)+1:3*(doa_ob_l(n,4)-1)+3) = ss_ini_pos_global_coord;
        x_ini(3*(doa_ob_l(n,4)-1)+1:3*(doa_ob_l(n,4)-1)+3) = [doa_ob_w(n,1);doa_ob_w(n,2);1/ss_pos_ini_doa_dist];
        
        g3d.x_gt(3*(doa_ob_l(n,4)-1)+1:3*(doa_ob_l(n,4)-1)+3) = [doa_gt_w(n,1);doa_gt_w(n,2);1/ss_pos_ini_doa_dist];
        
        lm_ini_pose_correspondance_3d(doa_ob_l(n,4),2) = doa_ob_l(n,3);
        %ss_ini_index = ss_ini_index+1;
        ss_ini_observed = [ss_ini_observed,doa_ob_l(n,4)];
    end
    
end
% save data into graph
g3d.x = x_ini;
g3d.lm_ini_pose_correspondance_3d = lm_ini_pose_correspondance_3d;

lm_ini_3d_xy_coord = zeros(ss_num,3);
for n=1:ss_num
    lm_ini_3d_xy_coord(n,:) = (x_ini(ss_num*3+7*(lm_ini_pose_correspondance_3d(n,2)-1)+1:ss_num*3+7*(lm_ini_pose_correspondance_3d(n,2)-1)+3) + ...
        (1/x_ini(3*(n-1)+3))*[cos(x_ini(3*(n-1)+2))*cos(x_ini(3*(n-1)+1));cos(x_ini(3*(n-1)+2))*sin(x_ini(3*(n-1)+1));sin(x_ini(3*(n-1)+2))])';
end

% plot the initialization of sound sources
figure(fig_3d_ob_num);
hold on;
plot3(lm_ini_3d_xy_coord(:,1),lm_ini_3d_xy_coord(:,2),lm_ini_3d_xy_coord(:,3),'LineStyle','none','Marker','o',...
    'MarkerSize',10,'MarkerEdgeColor','k','MarkerFaceColor','y');
hold off;
%% creation of edges 

% in 2D case:
edges_count = 1;

for n=2:size(robot_pose_2d,1)
    
    type_p_p = 'P';
    % measurement: noisy robot_pose_u control input observation
    measurement_p_p = robot_pose_u_ob_2d(n,:)';
    % information matrix [1/dist_sigma^2 0 0; 0 1/dits_sigma^2 0; 0 0 1/theta_sigma^2];
    information_p_p = diag([1/((odom_u_dist_std*odom_u_dist_std_conservative_factor)^2);...
        1/((odom_u_dist_std*odom_u_dist_std_conservative_factor)^2);1/((odom_u_theta_std*odom_u_theta_std_conservative_factor)^2)]);
    % index of previous time robot pose (starting index)
    fromIdx_p_p = 3*(n-2)+1 + 2*ss_num;
    % index of current time robot pose (starting index)
    toIdx_p_p = 3*(n-1)+1 + 2*ss_num;

    % fill the above information to "g" structure
    g2d.edges(edges_count).type = type_p_p;
    g2d.edges(edges_count).measurement = measurement_p_p;
    g2d.edges(edges_count).information = information_p_p;
    g2d.edges(edges_count).fromIdx = fromIdx_p_p;
    g2d.edges(edges_count).toIdx = toIdx_p_p;
    % update the edges_count
    edges_count = edges_count+1;
    
end

for n=1:size(doa_ob_l_2d,1)
    % (p-l) pose landmark constraits 
    % type of the constraint
    type_p_l = 'L';
    % measurement (noisy azim DOA observation)
    measurement_p_l = doa_ob_l_2d(n,1);
    % information matrix: (1/theta_sigma^2) (1x1 matrix)
    information_p_l = 1/((ss_doa_std_azi*ss_doa_std_azi_conservative_factor)^2);
    % starting index of sound source state vector (starting index)
    fromIdx_p_l = 2*doa_ob_l_2d(n,3)-1;
    % index of current time mic array/robot pose (starting index)
    toIdx_p_l = 3*doa_ob_l_2d(n,2)-2 + 2*ss_num;
    
    % fill the above information to "g" structure
    g2d.edges(edges_count).type = type_p_l;
    g2d.edges(edges_count).measurement = measurement_p_l;
    g2d.edges(edges_count).information = information_p_l;
    g2d.edges(edges_count).fromIdx = fromIdx_p_l;
    g2d.edges(edges_count).toIdx = toIdx_p_l;
    % update the edges_count
    edges_count = edges_count+1;
end



% in 3D case:
edges_count = 1;

for n=2:size(robot_pose,1)
    
    type_p_p = 'P';
    % measurement: noisy robot_pose_u control input observation
    measurement_p_p = robot_pose_u_ob_3d(n,:)';
    % information matrix 
    information_p_p = diag([1/((odom_u_dist_std*odom_u_dist_std_conservative_factor)^2);...
        1/((odom_u_dist_std*odom_u_dist_std_conservative_factor)^2);1/((odom_u_dist_std*odom_u_dist_std_conservative_factor)^2);...
        1/((odom_u_theta_std*odom_u_theta_std_conservative_factor)^2);1/((odom_u_theta_std*odom_u_theta_std_conservative_factor)^2);...
        1/((odom_u_theta_std*odom_u_theta_std_conservative_factor)^2)]);
    % index of previous time robot pose (starting index)
    fromIdx_p_p = 7*(n-2)+1 + 3*ss_num;
    % index of current time robot pose (starting index)
    toIdx_p_p = 7*(n-1)+1 + 3*ss_num;

    % fill the above information to "g" structure
    g3d.edges(edges_count).type = type_p_p;
    g3d.edges(edges_count).measurement = measurement_p_p;
    g3d.edges(edges_count).information = information_p_p;
    g3d.edges(edges_count).fromIdx = fromIdx_p_p;
    g3d.edges(edges_count).toIdx = toIdx_p_p;
    % update the edges_count
    edges_count = edges_count+1;
    
end

for n=1:size(doa_ob_l,1)
    % (p-l) pose landmark constraits 
    % type of the constraint
    type_p_l = 'L';
    % measurement (noisy azim DOA observation)
    measurement_p_l = doa_ob_l(n,1:2)';
    % information matrix: I*(1/theta_sigma^2) (2x2 matrix)
    information_p_l = diag([1/((ss_doa_std_azi*ss_doa_std_azi_conservative_factor)^2);...
        1/((ss_doa_std_ele*ss_doa_std_ele_conservative_factor)^2)]);
    % starting index of sound source state vector (starting index)
    fromIdx_p_l = 3*doa_ob_l(n,4)-2;
    % index of current time mic array/robot pose (starting index)
    toIdx_p_l = 7*doa_ob_l(n,3)-6 + 3*ss_num;
    
    % fill the above information to "g" structure
    g3d.edges(edges_count).type = type_p_l;
    g3d.edges(edges_count).measurement = measurement_p_l;
    g3d.edges(edges_count).information = information_p_l;
    g3d.edges(edges_count).fromIdx = fromIdx_p_l;
    g3d.edges(edges_count).toIdx = toIdx_p_l;
    % update the edges_count
    edges_count = edges_count+1;
end
%% initialization of state vector

%{

% initialize the mic state (pos: true position + big initialization noise
% for random pos, delay: 0 for absolute no knowledge of delay, drift: 0 for
% absolute no knowledge of drift (+randn*(1e-6) for avoiding singularity in
% slSLAM))
x_mic_init = [ss_pos+(mic_pos_init_noise*(rand(size(ss_pos))-0.5)*2),...
    zeros(M,1),zeros(M,1)+randn*(1e-6)];

% fix some mic position to avoid ambiguity
% 1st mic is reference, hence pos is (0,0) and 0 delay 0 drift
x_mic_init(1,:) = zeros(1,4);

% make sure the 2nd mic is on positive x axis
if(x_mic_init(2,1)<0)
    x_mic_init(2,1) = -1*x_mic_init(2,1);
end
x_mic_init(2,2) = 0;

% make sure the 3rd mic's y is positive
if(x_mic_init(4,2)<0)
    x_mic_init(4,2) = -1*x_mic_init(4,2);
end

% initialization of sound src postion. all (0,0) + noise to describe no
% knowledge about it.
% x_src_init = zeros(size(src_pos,1),2) + rand(size(src_pos,1),2);
x_src_init = g.x_gt(4*M+1:end,1) + randn(size(g.x_gt,1)-4*M,1)*...
    src_pos_init_noise;

% construct the initial state vector based on above.
g.x = [reshape(x_mic_init',4*M,1);x_src_init];

%}

%% create offset and dimension
% just to make it suitable for cyrill's code framework

% in 2D case:

% initialize two variables
offset = [];
dimension = [];

% construct two variables for the sound source state vecotr
for n=1:ss_num
    offset = [offset;(n-1)*2];
    dimension = [dimension;2];
end

% construct two variables for the robot pose state vecotr
for n=1:size(robot_pose_2d,1)
    offset = [offset;(n-1)*3 + 2*ss_num];
    dimension = [dimension;3];
end

% fill this two variables into "g" structure 
for n = 1:size(offset,1)
    g2d.idLookup(n).offset = offset(n);
    g2d.idLookup(n).dimension = dimension(n);
end  


% in 3D case:

% initialize two variables
offset = [];
dimension = [];

% construct two variables for the sound source state vecotr
for n=1:ss_num
    offset = [offset;(n-1)*3];
    dimension = [dimension;3];
end

% construct two variables for the robot pose state vecotr
for n=1:size(robot_pose,1)
    offset = [offset;(n-1)*7 + 3*ss_num];
    dimension = [dimension;7];
end

% fill this two variables into "g" structure 
for n = 1:size(offset,1)
    g3d.idLookup(n).offset = offset(n);
    g3d.idLookup(n).dimension = dimension(n);
end  


%% save the graph
g2d.ss_num = ss_num;
g2d.robot_pose_dist = robot_pose_dist;
g2d.doa_dist_max = doa_dist_max;

g3d.ss_num = ss_num;
g3d.robot_pose_dist = robot_pose_dist;
g3d.doa_dist_max = doa_dist_max;

% save the "g" structure
save('../data/mic_array.mat','g2d','g3d');

% that's all folks