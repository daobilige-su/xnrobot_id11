function pose3d_ypr = pose3d_quat_to_ypr(pose3d_quat)

% PLEASE TEST THESE TRANSFORMATION FUNCTION EVERY TIME WHEN MAKING NEW ONE.
% SOMETIME IT IS DIFFICULT TO DEBUG AND DIFFICULT TO THINK/IMAGINE THESE
% COMPLICATED 3D TRANSFORM
%
% this function returns trans+ypr from grid transformation matrix M
%
% test it by:
% pose3d_quat_to_ypr([[1,-2.9,-3],angle2quat(-1.21,-0.3,0)])
% we should ge [1;-2.9;-3;-1.21;-0.3;0];

t = [pose3d_quat(1);pose3d_quat(2);pose3d_quat(3)];
[y,p,r] = quat2angle([pose3d_quat(4),pose3d_quat(5),pose3d_quat(6),pose3d_quat(7)]);

pose3d_ypr = [t;y;p;r];



end
