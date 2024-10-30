function J=jacobian_idp2euc_2d(lm)

lm_x = lm(1);
lm_y = lm(2);
lm_theta = lm(3);
lm_idp = lm(4);

J = [ 1, 0, -sin(lm_theta)/lm_idp, -cos(lm_theta)/lm_idp^2;...
     0, 1,  cos(lm_theta)/lm_idp, -sin(lm_theta)/lm_idp^2];
end