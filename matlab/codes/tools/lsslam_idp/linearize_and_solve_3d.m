% performs one iteration of the Gauss-Newton algorithm
% each constraint is linearized and added to the Hessian

function [dx,H] = linearize_and_solve_3d(g)

nnz = nnz_of_graph_3d(g);

% allocate the sparse H and the vector b
H = spalloc(length(g.x), length(g.x), nnz);
b = zeros(length(g.x), 1);

needToAddPrior = true;

% compute the addend term to H and b for each of our constraints
disp('linearize and build system');
for eid = 1:length(g.edges)
  edge = g.edges(eid);

  % pose-pose constraint
  if (strcmp(edge.type, 'P') ~= 0)
    % edge.fromIdx and edge.toIdx describe the location of
    % the first element of the pose in the state vector
    % You should use also this index when updating the elements
    % of the H matrix and the vector b.
    % edge.measurement is the measurement
    % edge.information is the information matrix
    x1 = g.x(edge.fromIdx:edge.fromIdx+6);  % the first robot pose
    x2 = g.x(edge.toIdx:edge.toIdx+6);      % the second robot pose

    % Computing the error and the Jacobians
    % e the error vector
    % A Jacobian wrt x1
    % B Jacobian wrt x2
    [e, A, B] = linearize_pose_pose_constraint_3d(x1, x2, edge.measurement);
    

    % TODO: compute and add the term to H and b
    b(edge.fromIdx:edge.fromIdx+6) = (b(edge.fromIdx:edge.fromIdx+6)' + (e')*edge.information*A)';
    b(edge.toIdx:edge.toIdx+6) = (b(edge.toIdx:edge.toIdx+6)' + (e')*edge.information*B)';

    H(edge.fromIdx:edge.fromIdx+6,edge.fromIdx:edge.fromIdx+6) = H(edge.fromIdx:edge.fromIdx+6,edge.fromIdx:edge.fromIdx+6) + A'*edge.information*A;
    H(edge.fromIdx:edge.fromIdx+6,edge.toIdx:edge.toIdx+6) = H(edge.fromIdx:edge.fromIdx+6,edge.toIdx:edge.toIdx+6) + A'*edge.information*B;
    H(edge.toIdx:edge.toIdx+6,edge.fromIdx:edge.fromIdx+6) = H(edge.toIdx:edge.toIdx+6,edge.fromIdx:edge.fromIdx+6) + B'*edge.information*A;
    H(edge.toIdx:edge.toIdx+6,edge.toIdx:edge.toIdx+6) = H(edge.toIdx:edge.toIdx+6,edge.toIdx:edge.toIdx+6) + B'*edge.information*B;


  % pose-landmark constraint
  elseif (strcmp(edge.type, 'L') ~= 0)
    %continue;
    % edge.fromIdx and edge.toIdx describe the location of
    % the first element of the pose and the landmark in the state vector
    % You should use also this index when updating the elements
    % of the H matrix and the vector b.
    % edge.measurement is the measurement
    % edge.information is the information matrix
    x1 = g.x(edge.toIdx:edge.toIdx+6);   % the robot pose
    x2 = g.x(edge.fromIdx:edge.fromIdx+2);     % the landmark

    % Computing the error and the Jacobians
    % e the error vector
    % A Jacobian wrt x1
    % B Jacobian wrt x2
    [e, A, B] = linearize_pose_landmark_constraint_3d(x1, x2, edge.measurement);
    
    %edge.information = edge.information*0.001;
    
    % TODO: compute and add the term to H and b
    b(edge.fromIdx:edge.fromIdx+2) = (b(edge.fromIdx:edge.fromIdx+2)' + (e')*edge.information*A)';
    b(edge.toIdx:edge.toIdx+6) = (b(edge.toIdx:edge.toIdx+6)' + (e')*edge.information*B)';

    H(edge.fromIdx:edge.fromIdx+2,edge.fromIdx:edge.fromIdx+2) = H(edge.fromIdx:edge.fromIdx+2,edge.fromIdx:edge.fromIdx+2) + A'*edge.information*A;
    H(edge.fromIdx:edge.fromIdx+2,edge.toIdx:edge.toIdx+6) = H(edge.fromIdx:edge.fromIdx+2,edge.toIdx:edge.toIdx+6) + A'*edge.information*B;
    H(edge.toIdx:edge.toIdx+6,edge.fromIdx:edge.fromIdx+2) = H(edge.toIdx:edge.toIdx+6,edge.fromIdx:edge.fromIdx+2) + B'*edge.information*A;
    H(edge.toIdx:edge.toIdx+6,edge.toIdx:edge.toIdx+6) = H(edge.toIdx:edge.toIdx+6,edge.toIdx:edge.toIdx+6) + B'*edge.information*B;


    

  end
end

if (needToAddPrior)
  % TODO: add the prior for one pose of this edge
  % This fixes one node to remain at its current location

  H(g.ss_num*3+1:g.ss_num*3+7,g.ss_num*3+1:g.ss_num*3+7) = H(g.ss_num*3+1:g.ss_num*3+7,g.ss_num*3+1:g.ss_num*3+7)+eye(7);

  %needToAddPrior = false;
end
disp('solving system');

% TODO: solve the linear system, whereas the solution should be stored in dx
% Remember to use the backslash operator instead of inverting H

%Gaus-Newton
%for n=1:size(H,1)
%  H(n,n) = H(n,n);
%end

%L-M
for n=1:size(H,1)
  H(n,n) = H(n,n)+H(n,n)*0.1;
end


dx = H\(-b);

%dx = 0.0000000005*(-b);

end
