% Compute the error of a pose-landmark constraint
% x 3x1 vector (x,y,theta) of the robot pose
% l 2x1 vector (x,y) of the landmark
% z 2x1 vector (x,y) of the measurement, the position of the landmark in
%   the coordinate frame of the robot given by the vector x
%
% Output
% e 2x1 error of the constraint
% A 2x3 Jacobian wrt x
% B 2x2 Jacobian wrt l
function [e, A, B, C] = linearize_pose_landmark_constraint_3d(x, l, z, g, edge)

  % TODO compute the error and the Jacobians of the error
   
  % error
  %l_local = v2t(x)\[l;1];
  
  lm_idx = (edge.fromIdx+2)/3;
  
  anchor = g.x(g.ss_num*3+7*(g.lm_ini_pose_correspondance_3d(lm_idx,2)-1)+1:g.ss_num*3+7*(g.lm_ini_pose_correspondance_3d(lm_idx,2)-1)+3);
  anchor_xyz = anchor(1:3);
  
  lm_3d_xy_coord = anchor_xyz + (1/l(3))*[cos(l(2))*cos(l(1));cos(l(2))*sin(l(1));sin(l(2))];
  
  M_x = pose3d_quat_to_matrix(x);
  l_local = [M_x(1:3,1:3)',-M_x(1:3,1:3)'*M_x(1:3,4);0,0,0,1]*[lm_3d_xy_coord;1];
  
  %e = atan2(l_local(2),l_local(1))-z;
  e = [atan2(l_local(2),l_local(1));atan2(l_local(3),sqrt(l_local(1)^2+l_local(2)^2))]-z;
  
  % computation of A, de/dx1, x1 here is landmark
  %A = [-(l(2)-x(2))/((l(2)-x(2))^2+(l(1)-x(1))^2),+(l(1)-x(1))/((l(2)-x(2))^2+(l(1)-x(1))^2)];
  
  % computation of B, de/dx2, x2 here is robot pose
  %B = [(l(2)-x(2))/((l(2)-x(2))^2+(l(1)-x(1))^2),-(l(1)-x(1))/((l(2)-x(2))^2+(l(1)-x(1))^2),-1];
  [A,B,C] = jacobian_landmark3d_pose3d(x,l,anchor_xyz);
  
end