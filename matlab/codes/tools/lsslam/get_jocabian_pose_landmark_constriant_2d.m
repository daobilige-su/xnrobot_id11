syms p_x p_y p_theta l_x l_y z_theta real

x = [p_x;p_y;p_theta];
l = [l_x;l_y];
z = z_theta;

M = v2t(x);
l_local = [M(1:2,1:2)',-M(1:2,1:2)'*M(1:2,3);0,0,1]*[l;1];
l_theta = atan2(l_local(2),l_local(1));

M = v2t([0;0;z]);
e_t = t2v([M(1:2,1:2)',-M(1:2,1:2)'*M(1:2,3);0,0,1]*v2t([0;0;l_theta]));
e = e_t(3);



A = jacobian(e,[l_x l_y]);

B = jacobian(e,[p_x p_y p_theta]);

% then use clipboard('copy', char(A)) and clipboard('copy', char(B)) to
% copy and paste A and B to file jacobian_pose3d_pose3d. A and B are very
% long, they cannot be displayed on matlab console.