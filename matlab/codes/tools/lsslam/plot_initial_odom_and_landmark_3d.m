function plot_initial_odom_and_landmark_3d(robot_pose_ob_3d,doa_ob_l,doa_dist_max,figure_num)

% this function plot noisy odom and landmark in the initial graph.
fig_h = figure(figure_num);
set(fig_h, 'Position', [100, 100, 1000, 500]);

hold on;

% point for representing the coordinate origin
point_green = [1;0;0];
point_red = [0;1;0];
point_blue = [0;0;1];
point_origin = [0;0;0];

for n=1:size(robot_pose_ob_3d,1)
    
    point_green_transformed_hom = transform_matrix_from_trans_ypr(robot_pose_ob_3d(n,:))*[point_green;1];
    point_green_transformed = point_green_transformed_hom(1:3);
    
    point_red_transformed_hom = transform_matrix_from_trans_ypr(robot_pose_ob_3d(n,:))*[point_red;1];
    point_red_transformed = point_red_transformed_hom(1:3);
    
    point_blue_transformed_hom = transform_matrix_from_trans_ypr(robot_pose_ob_3d(n,:))*[point_blue;1];
    point_blue_transformed = point_blue_transformed_hom(1:3);
    
    point_origin_transformed_hom = transform_matrix_from_trans_ypr(robot_pose_ob_3d(n,:))*[point_origin;1];
    point_origin_transformed = point_origin_transformed_hom(1:3);
    
    plot3([point_origin_transformed(1),point_green_transformed(1)],[point_origin_transformed(2),point_green_transformed(2)],...
        [point_origin_transformed(3),point_green_transformed(3)],'Color','g');
    plot3([point_origin_transformed(1),point_red_transformed(1)],[point_origin_transformed(2),point_red_transformed(2)],...
        [point_origin_transformed(3),point_red_transformed(3)],'Color','r');
    plot3([point_origin_transformed(1),point_blue_transformed(1)],[point_origin_transformed(2),point_blue_transformed(2)],...
        [point_origin_transformed(3),point_blue_transformed(3)],'Color','b');
    
    % doa_ob_l: Nx4:[azi_ob_l,ele_ob_l,p,s]
    
    for m = 1:size(doa_ob_l,1)
        if doa_ob_l(m,3)==n
            doa_ob_direction_point = doa_dist_max*[cos(doa_ob_l(m,2))*cos(doa_ob_l(m,1));cos(doa_ob_l(m,2))*sin(doa_ob_l(m,1));sin(doa_ob_l(m,2))];
            
            doa_ob_direction_point_transformed_hom = transform_matrix_from_trans_ypr(robot_pose_ob_3d(n,:))*[doa_ob_direction_point;1];
            doa_ob_direction_point_transformed = doa_ob_direction_point_transformed_hom(1:3);
            
            plot3([point_origin_transformed(1),doa_ob_direction_point_transformed(1)],[point_origin_transformed(2),doa_ob_direction_point_transformed(2)],...
                [point_origin_transformed(3),doa_ob_direction_point_transformed(3)],'Color','m');
        end
    end
    
    
end

xlabel('x position (m)');ylabel('y position (m)');zlabel('z position (m)');
grid on;axis equal;view(45,30);
hold off;

end