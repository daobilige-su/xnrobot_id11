function P_local=global_point_in_local_coord_tans_ypr(P,tans_ypr)

% PLEASE TEST THESE TRANSFORMATION FUNCTION EVERY TIME WHEN MAKING NEW ONE.
% SOMETIME IT IS DIFFICULT TO DEBUG AND DIFFICULT TO THINK/IMAGINE THESE
% COMPLICATED 3D TRANSFORM
% 
% test it by:
% global_point_in_local_coord_tans_ypr([1,0,0],[0,0,1,pi/2,0,0]). It should
% give us [0,-1,-1]
% global_point_in_local_coord_tans_ypr([1,0,0],[0,0,1,pi/2,pi/2,0]). It should
% give us [1,-1,0]
%
% In all transform, it is always 1st translate the origin of local coord
% with translation and then rotate the orientation of local frame. In other
% words, all translation is made in global frame.

if size(P,1)==1
    P = P';
end

P = [P;1];

M = inverse_transform_matrix_from_trans_ypr(tans_ypr(1),tans_ypr(2),tans_ypr(3),tans_ypr(4),tans_ypr(5),tans_ypr(6));

P_local = M*P;

P_local = P_local(1:3);

end