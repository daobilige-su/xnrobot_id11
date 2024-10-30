% plot a 2D SLAM graph
function plot_graph_2d(g, iteration,H, plot_obs_on)

if nargin<2
iteration = -1;
end

clf;
hold on;

[p, l] = get_poses_landmarks_2d(g);

if (length(l) > 0)
  landmarkIdxX = l+1;
  landmarkIdxY = l+2;
  
  lm_2d_xy_coord = zeros(g.ss_num,2);
  for n=1:g.ss_num
    lm_2d_xy_coord(n,:) = (g.x(g.ss_num*2+3*(g.lm_ini_pose_correspondance_2d(n,2)-1)+1:g.ss_num*2+3*(g.lm_ini_pose_correspondance_2d(n,2)-1)+2) + ...
        (1/g.x(2*(n-1)+2))*[cos(g.x(2*(n-1)+1));sin(g.x(2*(n-1)+1))])';
  end
  
  lm_ini_2d_xy_coord_gt = zeros(g.ss_num,2);
  for n=1:g.ss_num
    lm_ini_2d_xy_coord_gt(n,:) = (g.x_gt(g.ss_num*2+3*(g.lm_ini_pose_correspondance_2d(n,2)-1)+1:g.ss_num*2+3*(g.lm_ini_pose_correspondance_2d(n,2)-1)+2) + ...
        (1/g.x_gt(2*(n-1)+2))*[cos(g.x_gt(2*(n-1)+1));sin(g.x_gt(2*(n-1)+1))])';
  end
  
  plot(lm_2d_xy_coord(:,1), lm_2d_xy_coord(:,2), 'LineStyle','none','Marker','o', 'MarkerSize', 10,'MarkerEdgeColor','b','MarkerFaceColor','y');
  plot(lm_ini_2d_xy_coord_gt(:,1), lm_ini_2d_xy_coord_gt(:,2), 'LineStyle','none','Marker','o', 'MarkerSize', 10,'MarkerEdgeColor','b','MarkerFaceColor','g');
end

% point for representing the coordinate origin
point_green = [1;0];
point_red = [0;1];
point_origin = [0;0];

if (length(p) > 0)
  pIdxX = p+1;
  pIdxY = p+2;
  pIdxTheta = p+3;
  for n=1:size(pIdxX,1)
      
    robot_pose = [g.x(pIdxX(n)),g.x(pIdxY(n)),g.x(pIdxTheta(n))];
      
    point_green_transformed_hom = transform_matrix_from_pose_2d(robot_pose)*[point_green;1];
    point_green_transformed = point_green_transformed_hom(1:2);
    
    point_red_transformed_hom = transform_matrix_from_pose_2d(robot_pose)*[point_red;1];
    point_red_transformed = point_red_transformed_hom(1:2);
    
    point_origin_transformed_hom = transform_matrix_from_pose_2d(robot_pose)*[point_origin;1];
    point_origin_transformed = point_origin_transformed_hom(1:2);
      
    plot([point_origin_transformed(1),point_green_transformed(1)],[point_origin_transformed(2),point_green_transformed(2)],...
        'Color','g');
    plot([point_origin_transformed(1),point_red_transformed(1)],[point_origin_transformed(2),point_red_transformed(2)],...
        'Color','r');
    
    %plot(g.x(pIdxX), g.x(pIdxY), 'LineStyle','none','Marker','x', 'MarkerSize', 4,'MarkerEdgeColor','b');
  end
end

% draw line segments???
doa_ob_l = [];

if plot_obs_on
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
      
      doa_ob_l = [doa_ob_l;[edge.measurement',((edge.toIdx+2-g.ss_num*2)/3),((edge.fromIdx+1)/2)]];
    end
  end
  
%   linespointx = [poseEdgesP1(1,:); poseEdgesP2(1,:)];
%   linespointy = [poseEdgesP1(2,:); poseEdgesP2(2,:)];
% 
%   plot(linespointx, linespointy, 'r');
  for m=1:size(doa_ob_l,1)
    doa_ob_direction_point = g.doa_dist_max*[cos(doa_ob_l(m,1));sin(doa_ob_l(m,1))];
    
    robot_pose = g.x(g.ss_num*2+3*(doa_ob_l(m,2)-1)+1:g.ss_num*2+3*(doa_ob_l(m,2)-1)+3);

    doa_ob_direction_point_transformed_hom = transform_matrix_from_pose_2d(robot_pose)*[doa_ob_direction_point;1];
    doa_ob_direction_point_transformed = doa_ob_direction_point_transformed_hom(1:2);

    point_origin_transformed_hom = transform_matrix_from_pose_2d(robot_pose)*[point_origin;1];
    point_origin_transformed = point_origin_transformed_hom(1:3);
    
    plot([point_origin_transformed(1),doa_ob_direction_point_transformed(1)],[point_origin_transformed(2),doa_ob_direction_point_transformed(2)],...
        'Color','m');
  end
end

%plot(poseEdgesP1(1,:), poseEdgesP1(2,:), "r");

%if (columns(poseEdgesP1) > 0)
%end
%if (columns(landmarkEdges) > 0)
%end

% draw the uncertainty

P = inv(H);


for n=1:g.ss_num
    
  lm_start_idx = 2*(n-1)+1;
  anchor_start_idx = g.ss_num*2+3*(g.lm_ini_pose_correspondance_2d(n,2)-1)+1;

  lm_xy_coord = lm_2d_xy_coord(n,:)';
  
%   g.x(lm_start_idx:lm_start_idx+1)
%   g.x(anchor_start_idx:anchor_start_idx+1)
  
  J = jacobian_idp2euc_2d([g.x(anchor_start_idx:anchor_start_idx+1);g.x(lm_start_idx:lm_start_idx+1)]);
  
  lm_sigma = P([anchor_start_idx,anchor_start_idx+1,lm_start_idx,lm_start_idx+1],...
      [anchor_start_idx,anchor_start_idx+1,lm_start_idx,lm_start_idx+1]);
  
  drawprobellipse(lm_xy_coord,J*lm_sigma*J',0.997,'r');
end


grid on;
%xlim([-(g.mic_dis) g.mic_dis*(g.M_x-1)+(g.mic_dis)]);ylim([-(g.mic_dis) g.mic_dis*(g.M_y-1)+(g.mic_dis)]);
xlabel('x (m)');ylabel('y (m)');%legend('est. mic. pos.','est. src. pos.');
axis equal;
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
