function quat=ypr2quat(ypr)

% PLEASE TEST THESE TRANSFORMATION FUNCTION EVERY TIME WHEN MAKING NEW ONE.
% SOMETIME IT IS DIFFICULT TO DEBUG AND DIFFICULT TO THINK/IMAGINE THESE
% COMPLICATED 3D TRANSFORM
%
% this function returns quaternion from yaw pitch roll angle, this function
% should be the same as matlab's angle2quat() function. But that one need
% argument not to be symbolic. So when Jacobian needs to be computed 
% symbolically, use this one instead.
%
% test it by:
% angle2quat(pi/2,0,0)
% ypr2quat([pi/2,0,0])
% the result should be the same;

y=ypr(1);
p=ypr(2);
r=ypr(3);

qr = cos(r/2)*cos(p/2)*cos(y/2)+sin(r/2)*sin(p/2)*sin(y/2);
qx = sin(r/2)*cos(p/2)*cos(y/2)-cos(r/2)*sin(p/2)*sin(y/2);
qy = cos(r/2)*sin(p/2)*cos(y/2)+sin(r/2)*cos(p/2)*sin(y/2);
qz = cos(r/2)*cos(p/2)*sin(y/2)-sin(r/2)*sin(p/2)*cos(y/2);

quat = [qr;qx;qy;qz];


end