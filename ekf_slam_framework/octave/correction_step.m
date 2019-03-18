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
N=(size(mu,1)-3)/2;
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
		mu(2+2*landmarkId:3+2*landmarkId)=mu(1:2)+[z(i).range * cos(z(i).bearing+mu(3));z(i).range*sin(z(i).bearing+mu(3))];
		% Indicate in the observedLandmarks vector that this landmark has been observed
		observedLandmarks(landmarkId) = true;
	endif

	% TODO: Add the landmark measurement to the Z vector
	Z(2*i-1:2*i)=[z(i).range;z(i).bearing];
	% TODO: Use the current estimate of the landmark pose
	% to compute the corresponding expected measurement in expectedZ:
  
  delta=mu(2+2*landmarkId:3+2*landmarkId)-mu(1:2);
  delta_x=delta(1);
  delta_y=delta(2);
  q=delta'*delta;
  expectedZ(2*i-1:2*i)=[sqrt(q);normalize_angle(atan2(delta_y,delta_x)-mu(3))];
	% TODO: Compute the Jacobian Hi of the measurement function h for this observation
  Fxj=zeros(5,2*N+3);
  Fxj(1:3,1:3)=eye(3);
  Fxj(4:5,2*landmarkId+2:2*landmarkId+3)=eye(2);  
	Hi=(1/q)*[-sqrt(q)*delta_x, -sqrt(q)*delta_y, 0, sqrt(q)*delta_x, sqrt(q)*delta_y;delta_y,-delta_x,-q,-delta_y,delta_x]*Fxj;
	% Augment H with the new Hi
	H = [H;Hi];	
endfor

% TODO: Construct the sensor noise matrix Q with dimension 2m x 2m
Q=0.01*eye(2*m);
% TODO: Compute the Kalman gain with dimension 2N+3 x 2m
K=sigma*H'*inv(H*sigma*H'+Q);
% TODO: Compute the difference between the expected and recorded measurements.
% Remember to normalize the bearings after subtracting!
% (hint: use the normalize_all_bearings function available in tools)
ze=Z-expectedZ;
ze=normalize_all_bearings(ze);
% TODO: Finish the correction step by computing the new mu and sigma.
% Normalize theta in the robot pose.
mu=mu+K*ze;
mu(3)=normalize_angle(mu(3));
sigma=(eye(2*N+3)-K*H)*sigma;
end
