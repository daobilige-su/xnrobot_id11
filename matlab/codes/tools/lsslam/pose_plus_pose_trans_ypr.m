function pose_final = pose_plus_pose_trans_ypr(pose_ini,pose_u)

% PLEASE TEST THESE TRANSFORMATION FUNCTION EVERY TIME WHEN MAKING NEW ONE.
% SOMETIME IT IS DIFFICULT TO DEBUG AND DIFFICULT TO THINK/IMAGINE THESE
% COMPLICATED 3D TRANSFORM
%
% test it by pose_plus_pose_trans_ypr([0,0,1,pi/2,0,0],[0,0,0,0,pi/2,pi/2])
% and you should get [0,0,1,0,pi/2,0]


M_u = transform_matrix_from_trans_ypr(pose_u(1),pose_u(2),...
    pose_u(3),pose_u(4),pose_u(5),pose_u(6));

M_ini = transform_matrix_from_trans_ypr(pose_ini(1),pose_ini(2),...
    pose_ini(3),pose_ini(4),pose_ini(5),pose_ini(6));

% 1st rotate by M_ini than rotate by M_u,
% if a 3D point P: P_final = M_ini*M_u*P.
% watch out: M_u*M_ini does not equal to M_ini*M_u and M_u is relative
% to global, so we should start from the end, that is 1st multiply by M_u,
% then M_ini.
M_final = M_ini*M_u;

pose_final = transform_matrix_to_pose_trans_ypr(M_final);

end