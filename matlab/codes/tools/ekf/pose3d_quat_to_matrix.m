function M = pose3d_quat_to_matrix(pose_quat)

% this function return 3D transformation matrix from 3d quat pose

% PLEASE TEST THESE TRANSFORMATION FUNCTION EVERY TIME WHEN MAKING NEW ONE.
% SOMETIME IT IS DIFFICULT TO DEBUG AND DIFFICULT TO THINK/IMAGINE THESE
% COMPLICATED 3D TRANSFORM
%
% test it by:
% a = [2,-3,-2.09,-0.3,1.09,0.843];
% pose3d_quat_to_matrix([a(1:3) angle2quat(a(4),a(5),a(6))])
% you should get the same result by running:
% transform_matrix_from_trans_ypr(a)

x = pose_quat(1);
y = pose_quat(2);
z = pose_quat(3);
qr = pose_quat(4);
qx = pose_quat(5);
qy = pose_quat(6);
qz = pose_quat(7);

M = [qr^2+qx^2-qy^2-qz^2,   2*(qx*qy-qr*qz),        2*(qz*qx+qr*qy),        x;
     2*(qx*qy+qr*qz),       qr^2-qx^2+qy^2-qz^2,    2*(qy*qz-qr*qx),        y;
     2*(qz*qx-qr*qy),       2*(qy*qz+qr*qx),        qr^2-qx^2-qy^2+qz^2,    z;
     0,                     0,                      0,                      1];


end