% Computes the total error of the graph
function Fx = compute_global_error_3d(g)

Fx = 0;

% Loop over all edges
for eid = 1:length(g.edges)
  edge = g.edges(eid);

  % pose-pose constraint
  if (strcmp(edge.type, 'P') ~= 0)

    %x1 = v2t(g.x(edge.fromIdx:edge.fromIdx+1));  % the first robot pose
    %x2 = v2t(g.x(edge.toIdx:edge.toIdx+1));      % the second robot pose
    p1 = g.x(edge.fromIdx:edge.fromIdx+6); % v2t() transforms a 2D pose to 3x3 transform matrix
    p2 = g.x(edge.toIdx:edge.toIdx+6);
    
    M1 = transform_matrix_from_trans_ypr(pose3d_quat_to_ypr(p1));
    M2 = transform_matrix_from_trans_ypr(pose3d_quat_to_ypr(p2));

    %TODO compute the error of the constraint and add it to Fx.
    % Use edge.measurement and edge.information to access the
    % measurement and the information matrix respectively.
    e_ij = transform_matrix_to_pose_trans_ypr(inv(transform_matrix_from_trans_ypr(edge.measurement))*(inv(M1))*M2);
    %e_ij = (x2 - x1) - edge.measurement;
    e_ls_ij = e_ij' * edge.information * e_ij;
    Fx = Fx + e_ls_ij;


  % pose-landmark constraint
  elseif (strcmp(edge.type, 'L') ~= 0)
    %continue;
    l = g.x(edge.fromIdx:edge.fromIdx+2);  % the landmark
    x = g.x(edge.toIdx:edge.toIdx+6);      % the robot pose

    %TODO compute the error of the constraint and add it to Fx.
    % Use edge.measurement and edge.information to access the
    % measurement and the information matrix respectively.
    %v2t_x = v2t(x);
    %e_il = ((v2t_x(1:2,1:2))')*(l-x(1:2)) - edge.measurement;
    l_local = transform_matrix_from_trans_ypr(pose3d_quat_to_ypr(x))\[l;1];
    e_il = [atan2(l_local(2),l_local(1));atan2(l_local(3),sqrt(l_local(1)^2+l_local(2)^2))]-edge.measurement;
    
    e_ls_il = e_il' * edge.information * e_il;
    Fx = Fx + e_ls_il;

  end

end
