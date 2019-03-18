% this function solves the odometry calibration problem
% given a measurement matrix Z.
% We assume that the information matrix is the identity
% for each of the measurements
% Every row of the matrix contains
% z_i = [u'x, u'y, u'theta, ux, uy, ytheta]
% Z:	The measurement matrix
% X:	the calibration matrix
% returns the correction matrix X
function X = ls_calibrate_odometry(Z)
  
  E=inf;
  while (E>1)
    % initial solution (the identity transformation)
    X = eye(3); 

    % TODO: initialize H and b of the linear system
    H=zeros(9,9);
    b=zeros(1,9);
    for i=1:size(Z,1)
        J_i = jacobian(i, Z);
        e_i=error_function(i,X,Z);
        H=H+J_i'*J_i;
        b=b+e_i'*J_i;
    end
    b=b';
    deltaX=-inv(H)*b;
    X=X+[deltaX(1:3)';deltaX(4:6)';deltaX(7:9)'];
    
    E=0;
    for i=1:size(Z,1)
      e_i=error_function(i,X,Z);
      E=E+e_i'*e_i;
    end
  % TODO: loop through the measurements and update H and b
  % You may call the functions error_function and jacobian, see below
  % We assume that the information matrix is the identity.
  end
  % TODO: solve and update the solution
  
end

% this function computes the error of the i^th measurement in Z
% given the calibration parameters
% i:	the number of the measurement
% X:	the actual calibration parameters
% Z:	the measurement matrix, each row contains first the scan-match result
%       and then the motion reported by odometry
% e:	the error of the ith measurement
function e = error_function(i, X, Z)
  % TODO compute the error of each measurement
  e=Z(i,1:3)'-X*Z(i,4:6)';
end

% derivative of the error function for the ith measurement in Z
% i:	the measurement number
% Z:	the measurement matrix
% J:	the jacobian of the ith measurement
function J = jacobian(i, Z)
  % TODO compute the Jacobian
  J=zeros(3,9);
  J(1,1:3)=-Z(i,4:6);
  J(2,4:6)=-Z(i,4:6);
  J(3,7:9)=-Z(i,4:6);
  
end
