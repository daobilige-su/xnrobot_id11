function M = inverse_transform_matrix_from_trans_ypr(x,y,z,yaw,pitch,roll,compute_once)

% PLEASE TEST THESE TRANSFORMATION FUNCTION EVERY TIME WHEN MAKING NEW ONE.
% SOMETIME IT IS DIFFICULT TO DEBUG AND DIFFICULT TO THINK/IMAGINE THESE
% COMPLICATED 3D TRANSFORM
% 
% inverse of Rigid transformation matrix M from pose/frame(x,y,z,yaw,pitch,roll):
%
% we can understand that global point P has been transformed by the
% same moment that inverse of M transformed from global coordinate.
% If we want to get local point P' from Rigid transformation
% matrix M (local coord frame/ pose) and global point P, P'=inv(M)*P.
% if M is not given, and Pose (tans+ypr) is given, we have to obtain M
% first, unfortunately.
%
% This means that how in local frame/pose M, we can see point global point P as P';
%
% test this function by
% inverse_transform_matrix_from_trans_ypr(0,0,0,pi/2,0,0)*[1,0,0,1]', it
% should be [0,-1,0,1]
%
% for detailed computation, look at "A tutorial on SE(3) transformation parameterizations and
% on-manifold optimization" by MAPIR Group (P. 32)


if nargin<7
    compute_once = 1;
end

if ~compute_once
    Rz = [cos(yaw), -sin(yaw), 0;
          sin(yaw), cos(yaw),  0;
          0,      0,       1];

    Ry = [cos(pitch), 0, sin(pitch);
          0,          1, 0;
          -sin(pitch),0, cos(pitch)];

    Rx = [1, 0,         0;
          0, cos(roll), -sin(roll);
          0, sin(roll), cos(roll)];

    R = Rz*Ry*Rx;
else

    R = [cos(yaw)*cos(pitch), cos(yaw)*sin(pitch)*sin(roll)-sin(yaw)*cos(roll), cos(yaw)*sin(pitch)*cos(roll)+sin(yaw)*sin(roll);
         sin(yaw)*cos(pitch), sin(yaw)*sin(pitch)*sin(roll)+cos(yaw)*cos(roll), sin(yaw)*sin(pitch)*cos(roll)-cos(yaw)*sin(roll);
         -sin(pitch),         cos(pitch)*sin(roll),                             cos(pitch)*cos(roll)];
end

M = [[R', -(R')*[x;y;z]]; [0,0,0,1]];

end