function [zNorm] = normalize_all_bearings_rblm_2d(z)
% Go over the observations vector and normalize the bearings
% The expected format of z is [range; bearing; range; bearing; ...]

for i=2:2:length(z)
   z(i) = normalize_angle(z(i));
end
zNorm = z;
