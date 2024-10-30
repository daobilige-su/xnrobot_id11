function [mu, sigma] = prediction_step_2d(mu, sigma, u, u_dist_std, u_theta_std)
% Updates the belief concerning the robot pose according to the motion model,
% mu: 2N+3 x 1 vector representing the state mean
% sigma: 2N+3 x 2N+3 covariance matrix
% u: odometry reading (r1, t, r2)
% Use u.r1, u.t, and u.r2 to access the rotation and translation values

% TODO: Compute the new mu based on the noise-free (odometry-based) motion model
% Remember to normalize theta after the update (hint: use the function normalize_angle available in tools)


mu(1) = mu(1) + u.t*cos(mu(3)+u.r1);
mu(2) = mu(2) + u.t*sin(mu(3)+u.r1);
mu(3) = mu(3) + u.r1 + u.r2;
mu(3) = wrapToPi(mu(3));

% TODO: Compute the 3x3 Jacobian Gx of the motion model

G_t_x = [1,0,-1*u.t*sin(mu(3)+u.r1);
	 0,1,u.t*cos(mu(3)+u.r1);
	 0,0,1];

% TODO: Construct the full Jacobian G

G_t = [G_t_x, zeros(3,(size(mu,1)-3));
       zeros((size(mu,1)-3),3),eye(size(mu,1)-3)];
	
% Motion noise
%motionNoise = 0.1;
R3 = [u_dist_std^2, 0, 0; 
     0, u_dist_std^2, 0; 
     0, 0, u_theta_std^2];
R = zeros(size(sigma,1));
R(1:3,1:3) = R3;

% TODO: Compute the predicted sigma after incorporating the motion

sigma = G_t*sigma*G_t'+R;

end
