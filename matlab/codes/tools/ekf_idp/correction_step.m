function [mu, sigma, observedLandmarks] = correction_step(mu, sigma, z, observedLandmarks)
% Updates the belief, i. e., mu and sigma after observing landmarks, according to the sensor model
% The employed sensor model measures the range and bearing of a landmark
% mu: 2N+3 x 1 vector representing the state mean.
% The first 3 components of mu correspond to the current estimate of the robot pose [x; y; theta]
% The current pose estimate of the landmark with id = j is: [mu(2*j+2); mu(2*j+3)]
% sigma: 2N+3 x 2N+3 is the covariance matrix
% z: struct array containing the landmark observations.
% Each observation z(i) has an id z(i).id, a range z(i).range, and a bearing z(i).bearing
% The vector observedLandmarks indicates which landmarks have been observed
% at some point by the robot.
% observedLandmarks(j) is false if the landmark with id = j has never been observed before.

% Number of measurements in this time step
m = size(z, 2);

% Z: vectorized form of all measurements made in this time step: [range_1; bearing_1; range_2; bearing_2; ...; range_m; bearing_m]
% ExpectedZ: vectorized form of all expected measurements in the same form.
% They are initialized here and should be filled out in the for loop below
Z = zeros(m*2, 1);
expectedZ = zeros(m*2, 1);

% Iterate over the measurements and compute the H matrix
% (stacked Jacobian blocks of the measurement function)
% H will be 2m x 2N+3
H = [];

for i = 1:m
	% Get the id of the landmark corresponding to the i-th observation
	landmarkId = z(i).id;
	% If the landmark is obeserved for the first time:
	if(observedLandmarks(landmarkId)==false)
		% TODO: Initialize its pose in mu based on the measurement and the current robot pose:
		mu(2*landmarkId+2) = mu(1) + z(i).range*cos(z(i).bearing+mu(3));
		mu(2*landmarkId+3) = mu(2) + z(i).range*sin(z(i).bearing+mu(3));
				
		% Indicate in the observedLandmarks vector that this landmark has been observed
		observedLandmarks(landmarkId) = true;
    end

	% TODO: Add the landmark measurement to the Z vector
	Z(2*i-1) = z(i).range;
	Z(2*i) = z(i).bearing;
	
	% TODO: Use the current estimate of the landmark pose
	% to compute the corresponding expected measurement in expectedZ:
	expectedZ(2*i-1) = sqrt((mu(2*landmarkId+2)-mu(1))^2 + (mu(2*landmarkId+3)-mu(2))^2);
	expectedZ(2*i) = atan2((mu(2*landmarkId+3)-mu(2)),(mu(2*landmarkId+2)-mu(1))) - mu(3);

	% TODO: Compute the Jacobian Hi of the measurement function h for this observation
	delta = [(mu(2*landmarkId+2)-mu(1));
		 (mu(2*landmarkId+3)-mu(2))];
	q = delta'*delta;
	
	Hi_low = (1/q)*[-sqrt(q)*delta(1),-sqrt(q)*delta(2),0,sqrt(q)*delta(1),sqrt(q)*delta(2);
                    delta(2),-delta(1),-q,-delta(2),delta(1)];
	Hi = zeros(2,size(mu,1));
	Hi(:,1:3) = Hi_low(:,1:3);
	Hi(:,2*landmarkId+2:2*landmarkId+3) = Hi_low(:,4:5);
	
	% Augment H with the new Hi
	H = [H;Hi];	
end

% TODO: Construct the sensor noise matrix Q

Q = 0.01*eye(2*m);

% TODO: Compute the Kalman gain
K = sigma*(H')/(H*sigma*H'+Q);

% TODO: Compute the difference between the expected and recorded measurements.
% Remember to normalize the bearings after subtracting!
% (hint: use the normalize_all_bearings function available in tools)
Z_diff = normalize_all_bearings(Z - expectedZ);

% TODO: Finish the correction step by computing the new mu and sigma.
% Normalize theta in the robot pose.
mu = mu + K*Z_diff;
sigma = (eye(size(mu,1))-K*H)*sigma;

end
