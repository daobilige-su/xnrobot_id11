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
function [e, A, B] = linearize_pose_rblm_landmark_constraint_2d(x, l, z)

  % TODO compute the error and the Jacobians of the error
   
  % error
%   l_local = v2t(x)\[l;1];
%   e = atan2(l_local(2),l_local(1))-z;
  
  % TODO compute the error and the Jacobians of the error
  M = v2t(x);
  e = ((M(1:2,1:2))')*(l-x(1:2))-z;

  B = [-cos(x(3)) , -sin(x(3)) , -sin(x(3))*(l(1)-x(1))+cos(x(3))*(l(2)-x(2));
       sin(x(3)) , -cos(x(3)) , -cos(x(3))*(l(1)-x(1))-sin(x(3))*(l(2)-x(2))];

  A = [cos(x(3)) , sin(x(3));
       -sin(x(3)) , cos(x(3))];
  
  
end