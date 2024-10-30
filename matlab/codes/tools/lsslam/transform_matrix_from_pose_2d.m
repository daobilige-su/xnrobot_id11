function M = transform_matrix_from_pose_2d(pose_2d)

% PLEASE TEST THESE TRANSFORMATION FUNCTION EVERY TIME WHEN MAKING NEW ONE.
% SOMETIME IT IS DIFFICULT TO DEBUG AND DIFFICULT TO THINK/IMAGINE THESE
% COMPLICATED 3D TRANSFORM
%
% test it by 
% a = transform_matrix_from_pose_2d([1,1,pi/4])
% a*[1,0,1]', you should get [1.7071,1.7071,1]';

x = pose_2d(1);
y = pose_2d(2);
theta = pose_2d(3);

M = [cos(theta), -sin(theta), x;
     sin(theta), cos(theta),  y;
     0,          0,           1];


end