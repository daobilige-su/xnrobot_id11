syms mu_x mu_y mu_z mu_qr mu_qx mu_qy mu_qz u_x u_y u_z u_yaw u_pitch u_roll real

mu = [mu_x mu_y mu_z mu_qr mu_qx mu_qy mu_qz];
u = [u_x u_y u_z u_yaw u_pitch u_roll];

%M_x = pose3d_quat_to_matrix(x);
%l_local = [M_x(1:3,1:3)',-M_x(1:3,1:3)'*M_x(1:3,4);0,0,0,1]*[l;1];

%e = t2v(inv(v2t(z))*(inv(v2t(x1)))*v2t(x2));
func_ypr = jaco_transform_matrix_to_pose_trans_ypr(pose3d_quat_to_matrix(mu)*transform_matrix_from_trans_ypr(u));

func_quat = [func_ypr(1:3);ypr2quat(func_ypr(4:6))];

A = jacobian(func_quat,[mu_x mu_y mu_z mu_qr mu_qx mu_qy mu_qz]);

%B = jacobian(e,[e_x,e_y,e_z,e_qr,e_qx,e_qy,e_qz]);

% then use clipboard('copy', char(A)) and clipboard('copy', char(B)) to
% copy and paste A and B to file jacobian_pose3d_pose3d. A and B are very
% long, they cannot be displayed on matlab console.