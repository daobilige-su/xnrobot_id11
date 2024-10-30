function pose_u_2d = pose_minus_pose_2d(pose_final,pose_ini)

% PLEASE TEST THESE TRANSFORMATION FUNCTION EVERY TIME WHEN MAKING NEW ONE.
% SOMETIME IT IS DIFFICULT TO DEBUG AND DIFFICULT TO THINK/IMAGINE THESE
% COMPLICATED 3D TRANSFORM
%
% test it by:
% pose_minus_pose_2d([1,1,pi/4],[1,1,0])
% you should get [0,0,pi/4]


M_final = transform_matrix_from_pose_2d(pose_final);
M_ini = transform_matrix_from_pose_2d(pose_ini);

% since M_final = M_ini*M_u --> M_u = inv(M_ini)*M_final
M_u = M_ini\M_final;

pose_u_2d = pose_2d_from_transform_matrix(M_u);

