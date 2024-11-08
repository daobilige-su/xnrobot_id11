%% param

rosbagfile = '../2021-08-03-20-41-48.bag';

slam_dist_thr = 0.10;
slam_theta_thr = 10*(pi/180);

init_pose = [-5.77,-6.0,0];
init_cov = [1.0^2, 0, 0;
            0, 1.0^2, 0;
            0, 0, (10*(pi/180))^2];
lm_cov = 0.1^2;

laser_in_baselink_trans_ypr = [0.1; 0; 0; 0; 0; 0];
laser_in_baselink_M = transform_matrix_from_trans_ypr(laser_in_baselink_trans_ypr);

odom_laser_t_diff_max = 0.3;
laser_dist_max = 3;
laser_data_asso_dist_thr = 0.3;

odom_dist_cov = 0.01^2;
odom_theta_cov = (5*(pi/180))^2;
laser_dist_cov = 0.05^2;
laser_ang_cov = (2*(pi/180))^2;

lm_x = repmat((4:-1:-4)',1,4);
lm_y = repmat((4.2:-2.8:-4.2),9,1);

lm_x = reshape(lm_x,[],1);
lm_y = reshape(lm_y,[],1);
%% main

mu = zeros(3+4*9*2,1);
mu(1:3) = init_pose;
mu(4:2:end) = lm_x;
mu(5:2:end) = lm_y;

init_pose_M = transform_matrix_from_trans_ypr([init_pose(1);init_pose(2);0;init_pose(3);0;0]);

P = [init_cov,zeros(3,4*9*2);
     zeros(4*9*2,3),eye(4*9*2)*lm_cov];


bag = rosbag(rosbagfile);

mu_hist=[];
odom_pre = [];
odom_pre_hist = [];
laser_msg_latest = [];
laser_pc_filter_in_map=[];

figure(1); clf;
hold on;
axis equal; grid on;
xlim([-7, 7]); ylim([-7, 7]);

for n=1:size(bag.MessageList,1)
    
    % handle odom
    topic_name = string(bag.MessageList{n,'Topic'});
    if strcmp(topic_name,'/odom')
        
        msg = readMessages(bag, n);
        odom_t = msg{1,1}.Header.Stamp.Sec+msg{1,1}.Header.Stamp.Nsec*(1e-9);
        odom_q = [msg{1,1}.Pose.Pose.Orientation.W, msg{1,1}.Pose.Pose.Orientation.X, msg{1,1}.Pose.Pose.Orientation.Y, msg{1,1}.Pose.Pose.Orientation.Z];
        [odom_theta,~,~] = quat2angle(odom_q);
        odom = [msg{1,1}.Pose.Pose.Position.X;msg{1,1}.Pose.Pose.Position.Y;odom_theta];
        
        lm_obs_id = zeros(4*9,1);
        z = [];
        
        if isempty(odom_pre)
            odom_pre = odom;
            odom_pre_M = transform_matrix_from_trans_ypr([odom_pre(1);odom_pre(2);0;odom_pre(3);0;0]);
            odom_pre_in_map_M = init_pose_M*odom_pre_M;
            odom_pre_in_map_trans_ypr = transform_matrix_to_pose_trans_ypr(odom_pre_in_map_M);
            odom_pre_in_map = [odom_pre_in_map_trans_ypr(1);odom_pre_in_map_trans_ypr(2);odom_pre_in_map_trans_ypr(4)];
            odom_pre_hist = [odom_pre_hist,odom_pre_in_map];
            mu_hist = [mu_hist, mu(1:3)];
        else
            odom_M = transform_matrix_from_trans_ypr([odom(1);odom(2);0;odom(3);0;0]);
            odom_pre_M = transform_matrix_from_trans_ypr([odom_pre(1);odom_pre(2);0;odom_pre(3);0;0]);
            
            odom_diff_M = odom_pre_M\odom_M;
            odom_diff_trans_ypr = transform_matrix_to_pose_trans_ypr(odom_diff_M);
            odom_diff = [odom_diff_trans_ypr(1);odom_diff_trans_ypr(2);odom_diff_trans_ypr(4)];
            
            u = [];
            u.r1 = wrapToPi(atan2(odom_diff(2),odom_diff(1)));
            u.t = sqrt(odom_diff(1)^2+odom_diff(2)^2);
            u.r2 = wrapToPi(odom_diff(3) - u.r1);
            
            
            if sqrt(odom_diff(1)^2+odom_diff(2)^2)>slam_dist_thr || abs(odom_diff(3))>slam_theta_thr
                
                % prediction
                [mu, P] = prediction_step_2d(mu, P, u, sqrt(odom_dist_cov), sqrt(odom_theta_cov));
                mu(4:2:end) = lm_x;
                mu(5:2:end) = lm_y;
                P = [P(1:3,1:3), zeros(3,4*9*2);
                     zeros(4*9*2,3), eye(4*9*2)*lm_cov];
                
                if abs(odom_t - laser_msg_latest.t) < odom_laser_t_diff_max
                    laser_range = laser_msg_latest.msg.Ranges;
                    % get only effective laser range
                    % laser_range(laser_range<0 | laser_range>laser_dist_max)=0;
                    
                    laser_angle = laser_msg_latest.msg.AngleMin + (0:laser_msg_latest.msg.AngleIncrement:laser_msg_latest.msg.AngleIncrement*(size(laser_range,1)-1))';
                    
                    laser_pc = [laser_range.*cos(laser_angle),laser_range.*sin(laser_angle)];
                    laser_pc_filter = laser_pc(laser_range>0 & laser_range<laser_dist_max,:);
                    % laser_range_filter = laser_range(laser_range>0 & laser_range<laser_dist_max);
                    % laser_angle_filter = laser_angle(laser_range>0 & laser_range<laser_dist_max);
                    %figure;plot(laser_pc(:,1),laser_pc(:,2),'LineStyle','none','Marker','.');axis equal;
                    
                    % transform laser points to base_link robot frame
                    laser_pc_filter = laser_in_baselink_M*[laser_pc_filter'; zeros(1,size(laser_pc_filter,1)); ones(1,size(laser_pc_filter,1))];
                    laser_pc_filter = laser_pc_filter(1:2,:)';
                    laser_range_filter = sqrt(laser_pc_filter(:,1).^2+laser_pc_filter(:,2).^2);
                    laser_angle_filter = atan2(laser_pc_filter(:,2),laser_pc_filter(:,1));
                    
                    
                    if ~isempty(laser_pc_filter)
                        mu_rob_M = transform_matrix_from_trans_ypr([mu(1);mu(2);0;mu(3);0;0]);
                        
                        laser_pc_filter_in_map = mu_rob_M*[laser_pc_filter';zeros(1,size(laser_pc_filter,1));ones(1,size(laser_pc_filter,1))];
                        laser_pc_filter_in_map = laser_pc_filter_in_map(1:2,:)';
                        
                        z_idx = 0;
                        for i=1:size(laser_pc_filter_in_map,1)
                            pt = laser_pc_filter_in_map(i,:);
                            [min_v,min_idx] = min(sqrt((pt(1)-lm_x).^2+(pt(2)-lm_y).^2));
                            
                            if min_v<laser_data_asso_dist_thr
                                if lm_obs_id(min_idx)==0 % if lm is hit by more than 1 laser, only take the 1st one
                                
                                    z_idx=z_idx+1;
                                    z(z_idx).id = min_idx;
                                    z(z_idx).range = laser_range_filter(i);
                                    z(z_idx).bearing = laser_angle_filter(i);

                                    lm_obs_id(min_idx) = 1;
                                end
                            end
                        end
                        
                        if ~isempty(z)
                            % correction
                            [mu, P] = correction_step_rblm_2d(mu, P, z,sqrt(laser_ang_cov),sqrt(laser_dist_cov));
                            mu(4:2:end) = lm_x;
                            mu(5:2:end) = lm_y;
                            P = [P(1:3,1:3), zeros(3,4*9*2);
                                 zeros(4*9*2,3), eye(4*9*2)*lm_cov];
                        end
                    end
                end
                
                robot_cur_pose = mu(1:3);
                
                odom_pre = odom;
                odom_pre_M = transform_matrix_from_trans_ypr([odom_pre(1);odom_pre(2);0;odom_pre(3);0;0]);
                odom_pre_in_map_M = init_pose_M*odom_pre_M;
                odom_pre_in_map_trans_ypr = transform_matrix_to_pose_trans_ypr(odom_pre_in_map_M);
                odom_pre_in_map = [odom_pre_in_map_trans_ypr(1);odom_pre_in_map_trans_ypr(2);odom_pre_in_map_trans_ypr(4)];
                odom_pre_hist = [odom_pre_hist,odom_pre_in_map];
                mu_hist = [mu_hist, mu(1:3)];
                
                % plot system
                plot_state(mu, P, z, odom_pre_hist, [],laser_pc_filter_in_map, mu_hist);
            else
                mu_rob_M = transform_matrix_from_trans_ypr([mu(1);mu(2);0;mu(3);0;0]);
                robot_cur_pose_M = mu_rob_M*odom_diff_M;
                robot_cur_pose_trans_ypr = transform_matrix_to_pose_trans_ypr(robot_cur_pose_M);
                robot_cur_pose = [robot_cur_pose_trans_ypr(1);robot_cur_pose_trans_ypr(2);robot_cur_pose_trans_ypr(4)];
                
                drawrobot(robot_cur_pose(1:3), 'b', 3, 0.8, 0.8);
                legend('odom','ekf');
                axis equal; grid on;
                xlim([-7, 7]); ylim([-7, 7]);
                
            end
            
            pause(0.01);
            disp(robot_cur_pose');
        end
        
        
    end
    
    % handle laser
    if strcmp(topic_name,'/vrep/scan')
        msg = readMessages(bag, n);
        
        laser_msg_latest.t = msg{1,1}.Header.Stamp.Sec+msg{1,1}.Header.Stamp.Nsec*(1e-9);
        laser_msg_latest.msg = msg{1,1};
    end
    
    
end
