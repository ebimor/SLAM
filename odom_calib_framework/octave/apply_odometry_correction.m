% computes a calibrated vector of odometry measurements
% by applying the bias term to each line of the measurements
% X: 	3x3 matrix obtained by the calibration process
% U: 	Nx3 matrix containing the odometry measurements
% C:	Nx3 matrix containing the corrected odometry measurements	

function C = apply_odometry_correction(X, U)
  % TODO: compute the calibrated motion vector, try to vectorize
  C=zeros(size(U));
  for i=1:size(U,1)
      s=X*U(i,:)';
      C(i,:)=s';
  end
  
  
end
