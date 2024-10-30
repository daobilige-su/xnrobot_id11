% Computes the total error of the graph
function Fx = compute_global_error_2d(g)

Fx = 0;

% Loop over all edges
for eid = 1:length(g.edges)
  edge = g.edges(eid);

  % pose-pose constraint
  if (strcmp(edge.type, 'P') ~= 0)

    %x1 = v2t(g.x(edge.fromIdx:edge.fromIdx+1));  % the first robot pose
    %x2 = v2t(g.x(edge.toIdx:edge.toIdx+1));      % the second robot pose
    M1 = v2t(g.x(edge.fromIdx:edge.fromIdx+2)); % v2t() transforms a 2D pose to 3x3 transform matrix
    M2 = v2t(g.x(edge.toIdx:edge.toIdx+2));

    %TODO compute the error of the constraint and add it to Fx.
    % Use edge.measurement and edge.information to access the
    % measurement and the information matrix respectively.
    e_ij = t2v(inv(v2t(edge.measurement))*(inv(M1))*M2);
    %e_ij = (x2 - x1) - edge.measurement;
    e_ls_ij = e_ij' * edge.information * e_ij;
    Fx = Fx + e_ls_ij;


  % pose-landmark constraint
  elseif (strcmp(edge.type, 'L') ~= 0)
    l = g.x(edge.fromIdx:edge.fromIdx+1);  % the landmark
    x = g.x(edge.toIdx:edge.toIdx+2);      % the robot pose

    %TODO compute the error of the constraint and add it to Fx.
    % Use edge.measurement and edge.information to access the
    % measurement and the information matrix respectively.
    %v2t_x = v2t(x);
    %e_il = ((v2t_x(1:2,1:2))')*(l-x(1:2)) - edge.measurement;
%     l_local = v2t(x)\[l;1];
%     e_il = atan2(l_local(2),l_local(1))-edge.measurement;
    l_local = v2t(x)\[l;1];
    l_theta = atan2(l_local(2),l_local(1));

    e_t = t2v(v2t([0;0;edge.measurement])'*v2t([0;0;l_theta]));
    e_il = e_t(3);
    
    e_ls_il = e_il' * edge.information * e_il;
    Fx = Fx + e_ls_il;
    
  % rblm
  elseif (strcmp(edge.type, 'L_rblm') ~= 0)
    l = g.x(edge.fromIdx:edge.fromIdx+1);  % the landmark
    x = g.x(edge.toIdx:edge.toIdx+2);      % the robot pose

    %TODO compute the error of the constraint and add it to Fx.
    % Use edge.measurement and edge.information to access the
    % measurement and the information matrix respectively.
    %v2t_x = v2t(x);
    %e_il = ((v2t_x(1:2,1:2))')*(l-x(1:2)) - edge.measurement;
    %l_local = v2t(x)\[l;1];
    %e_il = atan2(l_local(2),l_local(1))-edge.measurement;
    
    % Attention: this type of error is risky: when expected bearing is -175
    % and observation is 175, real difference is 10, but the following will
    % give -350. In error computation of bearing only measurement, this
    % type of situation is very common.!!!
    M = v2t(x);
    e_il = ((M(1:2,1:2))')*(l-x(1:2)) - edge.measurement;
    e_ls_il = e_il' * edge.information * e_il;
    Fx = Fx + e_ls_il;

  end

end
