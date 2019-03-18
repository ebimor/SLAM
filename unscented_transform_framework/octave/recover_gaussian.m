function [mu, sigma] = recover_gaussian(sigma_points, w_m, w_c)
% This function computes the recovered Gaussian distribution (mu and sigma)
% given the sigma points (size: nx2n+1) and their weights w_m and w_c:
% w_m = [w_m_0, ..., w_m_2n], w_c = [w_c_0, ..., w_c_2n].
% The weight vectors are each 1x2n+1 in size,
% where n is the dimensionality of the distribution.

n=size(sigma_points,1);
% Try to vectorize your operations as much as possible
trans_points=sigma_points;
% TODO: compute mu
W_m=repmat(w_m,[n,1]);
mu=W_m.*trans_points;
mu=mu*ones(2*n+1,1);

% TODO: compute sigma
W_c=repmat(w_c,[n,1]);
sigma=(W_c.*(trans_points-mu))*(trans_points-mu)';


end
