function plot_initial_odom_and_landmark_2d(robot_pose_ob_2d,doa_ob_l_2d,doa_dist_max,rblm_rb_ob_l_2d,figure_num)

% this function plot noisy odom and landmark in the initial graph.
fig_h = figure(figure_num);
% set(fig_h, 'Position', [100, 100, 1000, 500]);

hold on;

% point for representing the coordinate origin
point_green = [1;0];
point_red = [0;1];
point_origin = [0;0];

for n=1:size(robot_pose_ob_2d,1)
    
    point_green_transformed_hom = transform_matrix_from_pose_2d(robot_pose_ob_2d(n,:))*[point_green;1];
    point_green_transformed = point_green_transformed_hom(1:2);
    
    point_red_transformed_hom = transform_matrix_from_pose_2d(robot_pose_ob_2d(n,:))*[point_red;1];
    point_red_transformed = point_red_transformed_hom(1:2);
    
    point_origin_transformed_hom = transform_matrix_from_pose_2d(robot_pose_ob_2d(n,:))*[point_origin;1];
    point_origin_transformed = point_origin_transformed_hom(1:2);
    
    plot([point_origin_transformed(1),point_green_transformed(1)],[point_origin_transformed(2),point_green_transformed(2)],...
        'Color','g');
    plot([point_origin_transformed(1),point_red_transformed(1)],[point_origin_transformed(2),point_red_transformed(2)],...
        'Color','r');

    
    % doa_ob_l_2d_2d: Nx4:[azi_ob_l_2d,p,s]
    
    for m = 1:size(doa_ob_l_2d,1)
        if doa_ob_l_2d(m,2)==n
            doa_ob_direction_point = doa_dist_max*[cos(doa_ob_l_2d(m,1));sin(doa_ob_l_2d(m,1))];
            
            doa_ob_direction_point_transformed_hom = transform_matrix_from_pose_2d(robot_pose_ob_2d(n,:))*[doa_ob_direction_point;1];
            doa_ob_direction_point_transformed = doa_ob_direction_point_transformed_hom(1:2);
            
            plot([point_origin_transformed(1),doa_ob_direction_point_transformed(1)],[point_origin_transformed(2),doa_ob_direction_point_transformed(2)],...
                'Color','m');
        end
    end
    
    % rblm observation plotting
    for m = 1:size(rblm_rb_ob_l_2d,1)
        if rblm_rb_ob_l_2d(m,3)==n
            rblm_ob_direction_point = rblm_rb_ob_l_2d(m,1)*[cos(rblm_rb_ob_l_2d(m,2));sin(rblm_rb_ob_l_2d(m,2))];
            
            rblm_ob_direction_point_transformed_hom = transform_matrix_from_pose_2d(robot_pose_ob_2d(n,:))*[rblm_ob_direction_point;1];
            rblm_ob_direction_point_transformed = rblm_ob_direction_point_transformed_hom(1:2);
            
            plot([point_origin_transformed(1),rblm_ob_direction_point_transformed(1)],[point_origin_transformed(2),rblm_ob_direction_point_transformed(2)],...
                'Color','k');
        end
    end
    
    
end

xlabel('x position (m)');ylabel('y position (m)');zlabel('z position (m)');
grid on;axis equal;
hold off;

end