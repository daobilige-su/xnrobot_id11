function [zNorm] = normalize_all_bearings_rblm_3d(z)
% Go over the observations vector and normalize the bearings
% The expected format of z is [range; bearing; range; bearing; ...]

for i=3:3:length(z)
    z(i-1) = normalize_angle(z(i-1));
    if abs(z(i-1))>pi
       if z(i-1)>0
           z(i-1) = -(2*pi-abs(z(i-1)));
       else
           z(i-1) = (2*pi-abs(z(i-1)));
       end
    end
    
    z(i) = normalize_angle(z(i));
    if abs(z(i))>pi
       if z(i)>0
           z(i) = -(2*pi-abs(z(i)));
       else
           z(i) = (2*pi-abs(z(i)));
       end
    end
end
zNorm = z;
