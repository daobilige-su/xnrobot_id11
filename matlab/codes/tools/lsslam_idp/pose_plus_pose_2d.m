function pose_final = pose_plus_pose_2d(pose_ini,pose_u)

% PLEASE TEST THESE TRANSFORMATION FUNCTION EVERY TIME WHEN MAKING NEW ONE.
% SOMETIME IT IS DIFFICULT TO DEBUG AND DIFFICULT TO THINK/IMAGINE THESE
% COMPLICATED 3D TRANSFORM
%
% test it by:
% a = pose_plus_pose_2d([1.034,-3.98,-1.0456],[-2.34,1.23,0.34])
% pose_minus_pose_2d(a,[1.034,-3.98,-1.0456])
% you should get u:[-2.34,1.23,0.34]'


M_u = transform_matrix_from_pose_2d(pose_u);
M_ini = transform_matrix_from_pose_2d(pose_ini);

% since M_final = M_ini*M_u --> M_u = inv(M_ini)*M_final
M_final = M_ini*M_u;

pose_final = pose_2d_from_transform_matrix(M_final);

