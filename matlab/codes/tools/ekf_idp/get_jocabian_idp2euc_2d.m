syms lm_x lm_y lm_theta lm_idp real

%mu = [mu_x mu_y mu_theta];

lm_x_coord = (1/lm_idp)*cos(lm_theta) + lm_x;
lm_y_coord = (1/lm_idp)*sin(lm_theta) + lm_y;

func = [lm_x_coord;lm_y_coord];

%M_x = pose3d_quat_to_matrix(x);
%l_local = [M_x(1:3,1:3)',-M_x(1:3,1:3)'*M_x(1:3,4);0,0,0,1]*[l;1];

%e = t2v(inv(v2t(z))*(inv(v2t(x1)))*v2t(x2));


A = jacobian(func,[lm_x lm_y lm_theta lm_idp]);

% B = jacobian(expectedZ,[lm_x lm_y lm_z]);

% then use clipboard('copy', char(A)) and clipboard('copy', char(B)) to
% copy and paste A and B to file jacobian_pose3d_pose3d. A and B are very
% long, they cannot be displayed on matlab console.