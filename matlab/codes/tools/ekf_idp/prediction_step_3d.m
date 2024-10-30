function [mu, sigma] = prediction_step_3d(mu, sigma, u, u_dist_std, u_theta_std)
% Updates the belief concerning the robot pose according to the motion model,
% mu: 2N+3 x 1 vector representing the state mean
% sigma: 2N+3 x 2N+3 covariance matrix
% u: odometry reading (r1, t, r2)
% Use u.r1, u.t, and u.r2 to access the rotation and translation values

% TODO: Compute the new mu based on the noise-free (odometry-based) motion model
% Remember to normalize theta after the update (hint: use the function normalize_angle available in tools)


%mu(1) = mu(1) + u.t*cos(mu(3)+u.r1);
%mu(2) = mu(2) + u.t*sin(mu(3)+u.r1);
%mu(3) = mu(3) + u.r1 + u.r2;
%mu(3) = normalize_angle(mu(3));

u = [u.x;u.y;u.z;u.yaw;u.pitch;u.roll];

mu(1:7) = pose3d_ypr_to_quat(transform_matrix_to_pose_trans_ypr(pose3d_quat_to_matrix(mu(1:7))*transform_matrix_from_trans_ypr(u)));

% TODO: Compute the 3x3 Jacobian Gx of the motion model

% G_t_x = [1,0,-1*u.t*sin(mu(3)+u.r1);
% 	 0,1,u.t*(mu(3)+u.r1);
% 	 0,0,1];
G_t_x = ekf_3d_jacobian_motion(mu(1:7),u);

% TODO: Construct the full Jacobian G

G_t = [G_t_x, zeros(7,(size(mu,1)-7));
       zeros((size(mu,1)-7),7),eye(size(mu,1)-7)];
	
% Motion noise
%motionNoise = 0.1;
R3 = [(u_dist_std^2)*eye(3),zeros(3,3);
     zeros(3,3),(u_theta_std^2)*eye(3)];
R = zeros(size(sigma,1));
J_ypr_quat = ekf_3d_jacobian_ypr_to_quat(u);
R(1:7,1:7) = J_ypr_quat*R3*J_ypr_quat';

% TODO: Compute the predicted sigma after incorporating the motion

sigma = G_t*sigma*G_t'+R;

end
