syms p_x p_y p_theta an_x an_y an_theta l_theta l_idp z_theta real

x = [p_x;p_y;p_theta];
l = [l_theta;l_idp];
anchor = [an_x;an_y;an_theta];
anchor_xy = anchor(1:2);

z = z_theta;
  
lm_2d_xy_coord = anchor_xy + (1/l(2))*[cos(l(1));sin(l(1))];
  
l_local = v2t(x)\[lm_2d_xy_coord;1];
e = atan2(l_local(2),l_local(1))-z;

%e = t2v(inv(v2t(z))*(inv(v2t(x1)))*v2t(x2));
%e = [atan2(l_local(2),l_local(1));atan2(l_local(3),sqrt(l_local(1)^2+l_local(2)^2))]-z;

A = jacobian(e,[l_theta,l_idp]);

B = jacobian(e,[p_x,p_y,p_theta]);

C = jacobian(e,[an_x,an_y,an_theta]);

% then use clipboard('copy', char(A)) and clipboard('copy', char(B)) to
% copy and paste A and B to file jacobian_pose3d_pose3d. A and B are very
% long, they cannot be displayed on matlab console.