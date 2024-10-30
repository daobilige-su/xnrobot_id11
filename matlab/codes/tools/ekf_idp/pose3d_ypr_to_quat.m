function pose3d_ypr = pose3d_ypr_to_quat(pose3d_ypr)

% PLEASE TEST THESE TRANSFORMATION FUNCTION EVERY TIME WHEN MAKING NEW ONE.
% SOMETIME IT IS DIFFICULT TO DEBUG AND DIFFICULT TO THINK/IMAGINE THESE
% COMPLICATED 3D TRANSFORM
%
% this function returns trans+ypr from grid transformation matrix M
%
% test it by:
% [y,p,r] = quat2angle([0.7071,0,0,-0.7071])
% pose3d_ypr_to_quat([[1,-2.9,-3],[y,p,r]])
% we should ge [1;-2.9;-3;0.7071;0;0;-0.7071];

t = [pose3d_ypr(1);pose3d_ypr(2);pose3d_ypr(3)];
quat = angle2quat(pose3d_ypr(4),pose3d_ypr(5),pose3d_ypr(6));

pose3d_ypr = [t;quat'];



end
