syms p_x p_y p_z p_qr p_qx p_qy p_qz an_x an_y an_z l_azim l_elev l_idp z_azim z_elev real

x = [p_x;p_y;p_z;p_qr;p_qx;p_qy;p_qz];
l = [l_azim;l_elev;l_idp];
anchor = [an_x;an_y;an_z];
anchor_xy = anchor(1:3);

z = [z_azim;z_elev];
  
lm_3d_xy_coord = anchor_xy + (1/l(3))*[cos(l(2))*cos(l(1));cos(l(2))*sin(l(1));sin(l(2))];
  
% l_local = v2t(x)\[lm_2d_xy_coord;1];
% e = atan2(l_local(2),l_local(1))-z;

M_x = pose3d_quat_to_matrix(x);
l_local = [M_x(1:3,1:3)',-M_x(1:3,1:3)'*M_x(1:3,4);0,0,0,1]*[lm_3d_xy_coord;1];

%e = atan2(l_local(2),l_local(1))-z;
e = [atan2(l_local(2),l_local(1));atan2(l_local(3),sqrt(l_local(1)^2+l_local(2)^2))]-z;

%e = t2v(inv(v2t(z))*(inv(v2t(x1)))*v2t(x2));
%e = [atan2(l_local(2),l_local(1));atan2(l_local(3),sqrt(l_local(1)^2+l_local(2)^2))]-z;

A = jacobian(e,[l_azim l_elev l_idp]);

B = jacobian(e,[p_x p_y p_z p_qr p_qx p_qy p_qz]);

C = jacobian(e,[an_x an_y an_z]);

% then use clipboard('copy', char(A)) and clipboard('copy', char(B)) to
% copy and paste A and B to file jacobian_pose3d_pose3d. A and B are very
% long, they cannot be displayed on matlab console.