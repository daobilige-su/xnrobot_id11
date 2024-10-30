function x=norm_quat_3d(x,ss_num)

% this function normalize the quaternion component in lsSLAM_3d.m's state
% vector g3d.x;
%
% test it by: 
% norm_quat_3d([1,-2,3.54,1,0,0,1]',0)
% should get [1,-2,3.54,0.7071,0,0,0.7071]'

pose_num = (size(x,1)-3*ss_num)/7;

for n=1:pose_num
    
    x(3*ss_num+7*(n-1)+4:3*ss_num+7*(n-1)+7) = x(3*ss_num+7*(n-1)+4:3*ss_num+7*...
        (n-1)+7)/sqrt(sum(x(3*ss_num+7*(n-1)+4:3*ss_num+7*(n-1)+7).^2));
    
end


end