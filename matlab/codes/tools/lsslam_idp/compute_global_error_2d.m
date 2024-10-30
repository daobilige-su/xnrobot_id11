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
    
    
    lm_idx = (edge.fromIdx+1)/2;
    lm_2d_xy_coord = g.x(g.ss_num*2+3*(g.lm_ini_pose_correspondance_2d(lm_idx,2)-1)+1:g.ss_num*2+3*(g.lm_ini_pose_correspondance_2d(lm_idx,2)-1)+2) + ...
        (1/l(2))*[cos(l(1));sin(l(1))];
    
    l_local = v2t(x)\[lm_2d_xy_coord;1];
    
    e_il = atan2(l_local(2),l_local(1))-edge.measurement;
    
    e_ls_il = e_il' * edge.information * e_il;
    Fx = Fx + e_ls_il;

  end

end
