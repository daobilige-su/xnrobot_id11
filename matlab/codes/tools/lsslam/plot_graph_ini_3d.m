% plot a 2D SLAM graph
function plot_graph_ini_3d(g, iteration)

if nargin<2
iteration = -1;
end

clf;
hold on;

[p, l] = get_poses_landmarks_3d(g);

if (length(l) > 0)
  landmarkIdxX = l+1;
  landmarkIdxY = l+2;
  landmarkIdxZ = l+3;
  plot3(g.x(landmarkIdxX), g.x(landmarkIdxY), g.x(landmarkIdxZ), 'LineStyle','none','Marker','o', 'MarkerSize', 10,'MarkerEdgeColor','b','MarkerFaceColor','y');
  plot3(g.x_gt(landmarkIdxX), g.x_gt(landmarkIdxY), g.x(landmarkIdxZ), 'LineStyle','none','Marker','o', 'MarkerSize', 10,'MarkerEdgeColor','b','MarkerFaceColor','g');
end

% point for representing the coordinate origin
point_green = [1;0;0];
point_red = [0;1;0];
point_blue = [0;0;1];
point_origin = [0;0;0];

robot_pose_ypr =zeros(length(p),6);

if (length(p) > 0)
  pIdxX = p+1;
  pIdxY = p+2;
  pIdxZ = p+3;
  pIdxqr = p+4;
  pIdxqx = p+5;
  pIdxqy = p+6;
  pIdxqz = p+7;
  
  for n=1:size(pIdxX,1)
    
    robot_pose = [g.x(pIdxX(n)),g.x(pIdxY(n)),g.x(pIdxZ(n)),g.x(pIdxqr(n)),g.x(pIdxqx(n)),g.x(pIdxqy(n)),g.x(pIdxqz(n))];
      
    [yaw,pitch,roll] = quat2angle(robot_pose(4:7));
    quat_angle = [yaw,pitch,roll];
    robot_pose_ypr(n,:) = [robot_pose(1:3) quat_angle];
    
    point_green_transformed_hom = transform_matrix_from_trans_ypr([robot_pose(1:3) quat_angle])*[point_green;1];
    point_green_transformed = point_green_transformed_hom(1:3);
    
    point_red_transformed_hom = transform_matrix_from_trans_ypr([robot_pose(1:3) quat_angle])*[point_red;1];
    point_red_transformed = point_red_transformed_hom(1:3);
    
    point_blue_transformed_hom = transform_matrix_from_trans_ypr([robot_pose(1:3) quat_angle])*[point_blue;1];
    point_blue_transformed = point_blue_transformed_hom(1:3);
    
    point_origin_transformed_hom = transform_matrix_from_trans_ypr([robot_pose(1:3) quat_angle])*[point_origin;1];
    point_origin_transformed = point_origin_transformed_hom(1:3);
      
    plot3([point_origin_transformed(1),point_green_transformed(1)],[point_origin_transformed(2),point_green_transformed(2)],...
      [point_origin_transformed(3),point_green_transformed(3)],'Color','g');
    plot3([point_origin_transformed(1),point_red_transformed(1)],[point_origin_transformed(2),point_red_transformed(2)],...
      [point_origin_transformed(3),point_red_transformed(3)],'Color','r');
    plot3([point_origin_transformed(1),point_blue_transformed(1)],[point_origin_transformed(2),point_blue_transformed(2)],...
      [point_origin_transformed(3),point_blue_transformed(3)],'Color','b');
    
    %plot(g.x(pIdxX), g.x(pIdxY), 'LineStyle','none','Marker','x', 'MarkerSize', 4,'MarkerEdgeColor','b');
  end

end



% draw line segments???
doa_ob_l = [];

if 1
  poseEdgesP1 = [];
  poseEdgesP2 = [];
  landmarkEdgesP1 = [];
  landmarkEdgesP2 = [];
  for eid = 1:length(g.edges)
    edge = g.edges(eid);
    if (strcmp(edge.type, 'P') ~= 0)
      poseEdgesP1 = [poseEdgesP1, g.x(edge.fromIdx:edge.fromIdx+1)];
      poseEdgesP2 = [poseEdgesP2, g.x(edge.toIdx:edge.toIdx+1)];
    elseif (strcmp(edge.type, 'L') ~= 0)
      landmarkEdgesP1 = [landmarkEdgesP1, g.x(edge.fromIdx:edge.fromIdx+1)];
      landmarkEdgesP2 = [landmarkEdgesP2, g.x(edge.toIdx:edge.toIdx+1)];
      
      doa_ob_l = [doa_ob_l;[edge.measurement',((edge.toIdx+6-g.ss_num*3)/7),((edge.fromIdx+2)/3)]];
    end
  end
  
  %linespointx = [poseEdgesP1(1,:); poseEdgesP2(1,:)];
  %linespointy = [poseEdgesP1(2,:); poseEdgesP2(2,:)];

  %plot(linespointx, linespointy, 'r');
  
  for m=1:size(doa_ob_l,1)
    doa_ob_direction_point = g.doa_dist_max*[cos(doa_ob_l(m,2))*cos(doa_ob_l(m,1));cos(doa_ob_l(m,2))*sin(doa_ob_l(m,1));sin(doa_ob_l(m,2))];

    doa_ob_direction_point_transformed_hom = transform_matrix_from_trans_ypr(robot_pose_ypr(doa_ob_l(m,3),:))*[doa_ob_direction_point;1];
    doa_ob_direction_point_transformed = doa_ob_direction_point_transformed_hom(1:3);

    point_origin_transformed_hom = transform_matrix_from_trans_ypr(robot_pose_ypr(doa_ob_l(m,3),:))*[point_origin;1];
    point_origin_transformed = point_origin_transformed_hom(1:3);
    
    plot3([point_origin_transformed(1),doa_ob_direction_point_transformed(1)],[point_origin_transformed(2),doa_ob_direction_point_transformed(2)],...
        [point_origin_transformed(3),doa_ob_direction_point_transformed(3)],'Color','m');
  end
end

%plot(poseEdgesP1(1,:), poseEdgesP1(2,:), "r");

%if (columns(poseEdgesP1) > 0)
%end
%if (columns(landmarkEdges) > 0)
%end

grid on;
%xlim([-(g.mic_dis) g.mic_dis*(g.M_x-1)+(g.mic_dis)]);ylim([-(g.mic_dis) g.mic_dis*(g.M_y-1)+(g.mic_dis)]);
xlabel('x (m)');ylabel('y (m)');%legend('est. mic. pos.','est. src. pos.');
axis equal; view(45,30);
hold off;

%figure(1, 'visible', 'on');
figure(1);
drawnow;
%pause(0.1);
%{
if (iteration >= 0)
  filename = ['../plots/lsslam_' num2str(iteration) '.png'];
  %print(filename, '-dpng');
  saveas(gcf, filename, 'png');
end
%}


end
