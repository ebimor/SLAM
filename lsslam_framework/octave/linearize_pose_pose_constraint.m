% Compute the error of a pose-pose constraint
% x1 3x1 vector (x,y,theta) of the first robot pose
% x2 3x1 vector (x,y,theta) of the second robot pose
% z 3x1 vector (x,y,theta) of the measurement
%
% You may use the functions v2t() and t2v() to compute
% a Homogeneous matrix out of a (x, y, theta) vector
% for computing the error.
%
% Output
% e 3x1 error of the constraint
% A 3x3 Jacobian wrt x1
% B 3x3 Jacobian wrt x2
function [e, A, B] = linearize_pose_pose_constraint(x1, x2, z)

  % TODO compute the error and the Jacobians of the error
  
  
  
  X1=v2t(x1);
  X2=v2t(x2);
  Z=v2t(z);
  
  theta=x1(3);
  
  R1=X1(1:2,1:2);
  R2=X2(1:2,1:2);
  R12=Z(1:2,1:2);
  
  e=t2v(inv(Z)*(inv(X1)*X2));
 
  
  A=[R12'*(R1'*([-1;0])),R12'*(R1'*([0;-1])),R12'*([-sin(theta),cos(theta);-cos(theta),-sin(theta)]*[x2(1:2)-x1(1:2)])];
  A=[A;[0,0,-1]];
  
  
  B=[R12'*(R1'*([1;0])),R12'*(R1'*([0;1])),[0;0]];
  B=[B;[0,0,1]];
  
  


end;
