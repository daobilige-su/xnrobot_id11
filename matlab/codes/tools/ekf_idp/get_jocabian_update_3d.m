syms mu_x mu_y mu_z mu_qr mu_qx mu_qy mu_qz lm_x lm_y lm_z lm_azim lm_elev lm_idp real

mu = [mu_x;mu_y;mu_z;mu_qr;mu_qx;mu_qy;mu_qz];
lm = [lm_x;lm_y;lm_z;lm_azim;lm_elev;lm_idp];

% M_x = pose3d_quat_to_matrix(mu(1:7));
% 
% l_local = [M_x(1:3,1:3)',-M_x(1:3,1:3)'*M_x(1:3,4);0,0,0,1]*[lm;1];
% Z = [atan2(l_local(2),l_local(1));atan2(l_local(3),sqrt(l_local(1)^2+l_local(2)^2))];

M_x = pose3d_quat_to_matrix(mu(1:7));
lm_global_xyz = (1/lm_idp)*[cos(lm_elev)*cos(lm_azim);cos(lm_elev)*sin(lm_azim);sin(lm_elev)]+lm(1:3);

l_local = [M_x(1:3,1:3)',-M_x(1:3,1:3)'*M_x(1:3,4);0,0,0,1]*[lm_global_xyz;1];
Z = [atan2(l_local(2),l_local(1));atan2(l_local(3),sqrt(l_local(1)^2+l_local(2)^2))];

%M_x = pose3d_quat_to_matrix(x);
%l_local = [M_x(1:3,1:3)',-M_x(1:3,1:3)'*M_x(1:3,4);0,0,0,1]*[l;1];

%e = t2v(inv(v2t(z))*(inv(v2t(x1)))*v2t(x2));


A = jacobian(Z,[mu_x mu_y mu_z mu_qr mu_qx mu_qy mu_qz]);

B = jacobian(Z,[lm_x lm_y lm_z lm_azim lm_elev lm_idp]);

% then use clipboard('copy', char(A)) and clipboard('copy', char(B)) to
% copy and paste A and B to file jacobian_pose3d_pose3d. A and B are very
% long, they cannot be displayed on matlab console.