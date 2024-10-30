function J=get_jacobian_motion_2d(mu,lm)

mu_x = mu(1);
mu_y = mu(2);
mu_theta = mu(3);

lm_x = lm(1);
lm_y = lm(2);
lm_theta = lm(3);
lm_idp = lm(4);

J = [ (lm_y - mu_y + sin(lm_theta)/lm_idp)/((lm_y - mu_y + sin(lm_theta)/lm_idp)^2 + (lm_x - mu_x + cos(lm_theta)/lm_idp)^2), -(lm_x - mu_x + cos(lm_theta)/lm_idp)/((lm_y - mu_y + sin(lm_theta)/lm_idp)^2 + (lm_x - mu_x + cos(lm_theta)/lm_idp)^2), -1, -(lm_y - mu_y + sin(lm_theta)/lm_idp)/((lm_y - mu_y + sin(lm_theta)/lm_idp)^2 + (lm_x - mu_x + cos(lm_theta)/lm_idp)^2), (lm_x - mu_x + cos(lm_theta)/lm_idp)/((lm_y - mu_y + sin(lm_theta)/lm_idp)^2 + (lm_x - mu_x + cos(lm_theta)/lm_idp)^2), ((cos(lm_theta)/(lm_idp*(lm_x - mu_x + cos(lm_theta)/lm_idp)) + (sin(lm_theta)*(lm_y - mu_y + sin(lm_theta)/lm_idp))/(lm_idp*(lm_x - mu_x + cos(lm_theta)/lm_idp)^2))*(lm_x - mu_x + cos(lm_theta)/lm_idp)^2)/((lm_y - mu_y + sin(lm_theta)/lm_idp)^2 + (lm_x - mu_x + cos(lm_theta)/lm_idp)^2), -((sin(lm_theta)/(lm_idp^2*(lm_x - mu_x + cos(lm_theta)/lm_idp)) - (cos(lm_theta)*(lm_y - mu_y + sin(lm_theta)/lm_idp))/(lm_idp^2*(lm_x - mu_x + cos(lm_theta)/lm_idp)^2))*(lm_x - mu_x + cos(lm_theta)/lm_idp)^2)/((lm_y - mu_y + sin(lm_theta)/lm_idp)^2 + (lm_x - mu_x + cos(lm_theta)/lm_idp)^2)];

end