function [mu, sigma, sigma_points] = prediction_step(mu, sigma, u)
% Updates the belief concerning the robot pose according to the motion model.
% mu: state vector containing robot pose and poses of landmarks obeserved so far
% Current robot pose = mu(1:3)
% Note that the landmark poses in mu are stacked in the order by which they were observed
% sigma: the covariance matrix of the system.
% u: odometry reading (r1, t, r2)
% Use u.r1, u.t, and u.r2 to access the rotation and translation values

% For computing lambda.
global scale;

% Compute sigma points
sigma_points = compute_sigma_points(mu, sigma);

% Dimensionality
n = length(mu);
% lambda
lambda = scale - n;

% TODO: Transform all sigma points according to the odometry command
% Remember to vectorize your operations and normalize angles
% Tip: the function normalize_angle also works on a vector (row) of angles
sigma_points(1:3,:)=sigma_points(1:3,:)+[u.t * cos(sigma_points(3,:)+ u.r1); u.t * sin(sigma_points(3,:)+u.r1);repmat([u.r1+u.r2],[1,2*n+1])];
sigma_points(3,:)=normalize_angle(sigma_points(3,:));
% Computing the weights for recovering the mean
wm = [lambda/scale, repmat(1/(2*scale),1,2*n)];
wc = wm;

% TODO: recover mu.
% Be careful when computing the robot's orientation (sum up the sines and
% cosines and recover the 'average' angle via atan2)
mu=sigma_points*wm';
cos_mu=cos(sigma_points(3,:))*wm';
sin_mu=sin(sigma_points(3,:))*wm';
mu(3)=atan2(sin_mu,cos_mu);

% TODO: Recover sigma. Again, normalize the angular difference
Xe=sigma_points-mu;
Xe(3,:)=normalize_angle(Xe(3,:));
sigma=(wc.*Xe)*Xe';

% Motion noise
motionNoise = 0.1;
R3 = [motionNoise, 0, 0; 
     0, motionNoise, 0; 
     0, 0, motionNoise/10];
R = zeros(size(sigma,1));
R(1:3,1:3) = R3;

% TODO: Add motion noise to sigma
sigma=sigma+R;

end
