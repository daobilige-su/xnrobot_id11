syms p_x p_y p_theta l_x l_y z_r z_theta real

x = [p_x;p_y;p_theta];
l = [l_x;l_y];

z = [z_r;z_theta];

%M_x = pose3d_quat_to_matrix(x);
%l_local = [M_x(1:3,1:3)',-M_x(1:3,1:3)'*M_x(1:3,4);0,0,0,1]*[l;1];

%e = t2v(inv(v2t(z))*(inv(v2t(x1)))*v2t(x2));
%e = [atan2(l_local(2),l_local(1));atan2(l_local(3),sqrt(l_local(1)^2+l_local(2)^2))]-z;
l_local = v2t(x)\[l;1];
e = [sqrt(sum(l_local(1:2).^2));atan2(l_local(2),l_local(1))]-z;

A = jacobian(e,[l_x;l_y]);

B = jacobian(e,[p_x;p_y;p_theta]);

% then use clipboard('copy', char(A)) and clipboard('copy', char(B)) to
% copy and paste A and B to file jacobian_pose3d_pose3d. A and B are very
% long, they cannot be displayed on matlab console.