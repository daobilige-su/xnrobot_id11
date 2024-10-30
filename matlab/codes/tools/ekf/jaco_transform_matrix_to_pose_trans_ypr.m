function pose = jaco_transform_matrix_to_pose_trans_ypr(M)

% PLEASE TEST THESE TRANSFORMATION FUNCTION EVERY TIME WHEN MAKING NEW ONE.
% SOMETIME IT IS DIFFICULT TO DEBUG AND DIFFICULT TO THINK/IMAGINE THESE
% COMPLICATED 3D TRANSFORM
%
% this function returns trans+ypr from grid transformation matrix M
%
% test it by:
% a = transform_matrix_from_trans_ypr(1,2,3.5,1.09,-0.5,-0.135);
% transform_matrix_to_pose_trans_ypr(a)



x = M(1,4);
y = M(2,4);
z = M(3,4);

% atan2(y,x) returns values from -pi to pi. it handles the case of x<0 
% automatically 
pitch = atan2(-M(3,1),sqrt(M(1,1)^2+M(2,1)^2));

% in fact, in real function, the situation of pitch==+pi/2 or -pi/2 needs
% to be discussed seperately.
% But here, in order to get a simple jacobian, this two singularity
% situations are ignored. Probably later we might get a 'nan' error when we
% compute its jacobian when the pitch value reaches these two polarity (pi/2 or -pi/2).
% In this case we might need to add a small jitter to pitch value to avoid
% this nan error.
yaw = atan2(M(2,1),M(1,1));
roll = atan2(M(3,2),M(3,3));


pose = [x;y;z;yaw;pitch;roll];

end