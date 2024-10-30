function pose_u = pose_minus_pose_trans_ypr(pose_final,pose_ini)

% PLEASE TEST THESE TRANSFORMATION FUNCTION EVERY TIME WHEN MAKING NEW ONE.
% SOMETIME IT IS DIFFICULT TO DEBUG AND DIFFICULT TO THINK/IMAGINE THESE
% COMPLICATED 3D TRANSFORM
%
% this function get the difference of two poses. In other word, pose_ini
% plus pose_u should get pose pose final. Now we are going to get pose_u =
% pose_final - pose_ini.
% since pose_final = pose_ini + pose_u
% pose_u = inv(pos_ini) + pose_final --> pose combination
%
% test it by:
% pose_ini = [1,3,5,1.2,-0.2,-0.135]'
% pose_u = [2,-3,-0.4,-1.75,0.2,-0.285]'
% M_final = transform_matrix_from_trans_ypr(pose_ini(1),pose_ini(2),pose_ini(3),pose_ini(4),pose_ini(5),pose_ini(6))*...
% transform_matrix_from_trans_ypr(pose_u(1),pose_u(2),pose_u(3),pose_u(4),pose_u(5),pose_u(6));
% pose_final = transform_matrix_to_pose_trans_ypr(M_final)
% pose_minus_pose_trans_ypr(pose_final,pose_ini)
% you should get pose_u


M_final = transform_matrix_from_trans_ypr(pose_final(1),pose_final(2),...
    pose_final(3),pose_final(4),pose_final(5),pose_final(6));

M_ini_inv = inverse_transform_matrix_from_trans_ypr(pose_ini(1),pose_ini(2),...
    pose_ini(3),pose_ini(4),pose_ini(5),pose_ini(6));

% M = M1 * M2 (the order is very important), it means that we need to
% rotate M2 1st and then rotate M1. M1 is relative frame to M2. For
% example:
% a = M_final*M_ini_inv
% b = M_ini_inv*M_final
% are different.
%
% since M_final = M_final_plus_M_ini_inv*M_ini -->
% M_final_minus_M_ini = M_final*M_ini_inv;

M_final_minus_M_ini = M_ini_inv*M_final;

pose_u = transform_matrix_to_pose_trans_ypr(M_final_minus_M_ini);

end