function pose = pose_2d_from_transform_matrix(M)

% PLEASE TEST THESE TRANSFORMATION FUNCTION EVERY TIME WHEN MAKING NEW ONE.
% SOMETIME IT IS DIFFICULT TO DEBUG AND DIFFICULT TO THINK/IMAGINE THESE
% COMPLICATED 3D TRANSFORM

% test it by 
% a = transform_matrix_from_pose_2d([3.5,-4.002,-0.9456])
% pose_2d_from_transform_matrix(a)


x = M(1,3);
y = M(2,3);

theta = atan2(M(2,1),M(1,1));

pose = [x;y;theta];



end