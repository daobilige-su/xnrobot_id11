syms e_x e_y e_z e_qr e_qx e_qy e_qz e_lx e_ly e_lz m_azi m_ele real

x = [e_x;e_y;e_z;e_qr;e_qx;e_qy;e_qz];
l = [e_lx;e_ly;e_lz];

z = [m_azi;m_ele];

M_x = pose3d_quat_to_matrix(x);
l_local = [M_x(1:3,1:3)',-M_x(1:3,1:3)'*M_x(1:3,4);0,0,0,1]*[l;1];

%e = t2v(inv(v2t(z))*(inv(v2t(x1)))*v2t(x2));
e = [atan2(l_local(2),l_local(1));atan2(l_local(3),sqrt(l_local(1)^2+l_local(2)^2))]-z;

A = jacobian(e,[e_lx,e_ly,e_lz]);

B = jacobian(e,[e_x,e_y,e_z,e_qr,e_qx,e_qy,e_qz]);

% then use clipboard('copy', char(A)) and clipboard('copy', char(B)) to
% copy and paste A and B to file jacobian_pose3d_pose3d. A and B are very
% long, they cannot be displayed on matlab console.