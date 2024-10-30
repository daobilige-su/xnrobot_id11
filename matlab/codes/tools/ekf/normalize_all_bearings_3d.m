function [zNorm] = normalize_all_bearings_3d(z)
% Go over the observations vector and normalize the bearings
% The expected format of z is [range; bearing; range; bearing; ...]

for i=1:length(z)
   z(i) = normalize_angle(z(i));
end
zNorm = z;
