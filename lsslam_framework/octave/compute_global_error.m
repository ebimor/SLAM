% Computes the total error of the graph
function Fx = compute_global_error(g)

Fx = 0;

% Loop over all edges
for eid = 1:length(g.edges)
  edge = g.edges(eid);

  % pose-pose constraint
  if (strcmp(edge.type, 'P') != 0)

    X1 = v2t(g.x(edge.fromIdx:edge.fromIdx+2));  % the first robot pose
    X2 = v2t(g.x(edge.toIdx:edge.toIdx+2));      % the second robot pose

    %TODO compute the error of the constraint and add it to Fx.
    % Use edge.measurement and edge.information to access the
    % measurement and the information matrix respectively.
    Z=v2t(edge.measurement);
    e=t2v(inv(Z)*(inv(X1)*X2));

  % pose-landmark constraint
  elseif (strcmp(edge.type, 'L') != 0)
    X = g.x(edge.fromIdx:edge.fromIdx+2);  % the robot pose
    L = g.x(edge.toIdx:edge.toIdx+1);      % the landmark

    %TODO compute the error of the constraint and add it to Fx.
    % Use edge.measurement and edge.information to access the
    % measurement and the information matrix respectively.
    S=v2t(X);
    R=S(1:2,1:2);
    T=S(1:2,3);
    e=R'*(L-T)-edge.measurement;
  end

  omega=edge.information;
  Fx+=e'*omega*e;
end
