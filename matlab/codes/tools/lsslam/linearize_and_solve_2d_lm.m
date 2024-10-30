% performs one iteration of the Gauss-Newton algorithm
% each constraint is linearized and added to the Hessian

function [dx,H, lm_rau,lm_F_new,lm_lamda] = linearize_and_solve_2d_lm(g,lm_F_res,lm_lamda,lm_tau,i)

nnz = nnz_of_graph_2d(g);

% allocate the sparse H and the vector b
H = spalloc(length(g.x), length(g.x), nnz);
%H = zeros(length(g.x));
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
    x1 = g.x(edge.fromIdx:edge.fromIdx+2);  % the first robot pose
    x2 = g.x(edge.toIdx:edge.toIdx+2);      % the second robot pose

    % Computing the error and the Jacobians
    % e the error vector
    % A Jacobian wrt x1
    % B Jacobian wrt x2
    [e, A, B] = linearize_pose_pose_constraint_2d(x1, x2, edge.measurement);

    if edge.fromIdx==g.ss_num*2+g.rblm_num*2+1
      A = zeros(3,3);
    end
    

    % TODO: compute and add the term to H and b
    b(edge.fromIdx:edge.fromIdx+2) = (b(edge.fromIdx:edge.fromIdx+2)' + (e')*edge.information*A)';
    b(edge.toIdx:edge.toIdx+2) = (b(edge.toIdx:edge.toIdx+2)' + (e')*edge.information*B)';

    H(edge.fromIdx:edge.fromIdx+2,edge.fromIdx:edge.fromIdx+2) = H(edge.fromIdx:edge.fromIdx+2,edge.fromIdx:edge.fromIdx+2) + A'*edge.information*A;
    H(edge.fromIdx:edge.fromIdx+2,edge.toIdx:edge.toIdx+2) = H(edge.fromIdx:edge.fromIdx+2,edge.toIdx:edge.toIdx+2) + A'*edge.information*B;
    H(edge.toIdx:edge.toIdx+2,edge.fromIdx:edge.fromIdx+2) = H(edge.toIdx:edge.toIdx+2,edge.fromIdx:edge.fromIdx+2) + B'*edge.information*A;
    H(edge.toIdx:edge.toIdx+2,edge.toIdx:edge.toIdx+2) = H(edge.toIdx:edge.toIdx+2,edge.toIdx:edge.toIdx+2) + B'*edge.information*B;

    

  % pose-landmark constraint
  elseif (strcmp(edge.type, 'L') ~= 0)
    % edge.fromIdx and edge.toIdx describe the location of
    % the first element of the pose and the landmark in the state vector
    % You should use also this index when updating the elements
    % of the H matrix and the vector b.
    % edge.measurement is the measurement
    % edge.information is the information matrix
    x1 = g.x(edge.toIdx:edge.toIdx+2);   % the robot pose
    x2 = g.x(edge.fromIdx:edge.fromIdx+1);     % the landmark

    % Computing the error and the Jacobians
    % e the error vector
    % A Jacobian wrt x1
    % B Jacobian wrt x2
    [e, A, B] = linearize_pose_landmark_constraint_2d(x1, x2, edge.measurement);

    if edge.toIdx==g.ss_num*2+g.rblm_num*2+1
      B = zeros(2,3);
    end
    
    
    % TODO: compute and add the term to H and b
    b(edge.fromIdx:edge.fromIdx+1) = (b(edge.fromIdx:edge.fromIdx+1)' + (e')*edge.information*A)';
    b(edge.toIdx:edge.toIdx+2) = (b(edge.toIdx:edge.toIdx+2)' + (e')*edge.information*B)';

    H(edge.fromIdx:edge.fromIdx+1,edge.fromIdx:edge.fromIdx+1) = H(edge.fromIdx:edge.fromIdx+1,edge.fromIdx:edge.fromIdx+1) + A'*edge.information*A;
    H(edge.fromIdx:edge.fromIdx+1,edge.toIdx:edge.toIdx+2) = H(edge.fromIdx:edge.fromIdx+1,edge.toIdx:edge.toIdx+2) + A'*edge.information*B;
    H(edge.toIdx:edge.toIdx+2,edge.fromIdx:edge.fromIdx+1) = H(edge.toIdx:edge.toIdx+2,edge.fromIdx:edge.fromIdx+1) + B'*edge.information*A;
    H(edge.toIdx:edge.toIdx+2,edge.toIdx:edge.toIdx+2) = H(edge.toIdx:edge.toIdx+2,edge.toIdx:edge.toIdx+2) + B'*edge.information*B;


%     if (needToAddPrior)
%       % TODO: add the prior for one pose of this edge
%       % This fixes one node to remain at its current location
% 
%       H(g.ss_num*2+1:g.ss_num*2+3,g.ss_num*2+1:g.ss_num*2+3) = eye(3);
%       
%       %needToAddPrior = false;
%     end
  elseif (strcmp(edge.type, 'L_rblm') ~= 0)
    % edge.fromIdx and edge.toIdx describe the location of
    % the first element of the pose and the landmark in the state vector
    % You should use also this index when updating the elements
    % of the H matrix and the vector b.
    % edge.measurement is the measurement
    % edge.information is the information matrix
    x1 = g.x(edge.toIdx:edge.toIdx+2);   % the robot pose
    x2 = g.x(edge.fromIdx:edge.fromIdx+1);     % the landmark

    % Computing the error and the Jacobians
    % e the error vector
    % A Jacobian wrt x1
    % B Jacobian wrt x2
    [e, A, B] = linearize_pose_rblm_landmark_constraint_2d(x1, x2, edge.measurement);

    if edge.toIdx==g.ss_num*2+g.rblm_num*2+1
      B = zeros(2,3);
    end
    
    
    % TODO: compute and add the term to H and b
    b(edge.fromIdx:edge.fromIdx+1) = (b(edge.fromIdx:edge.fromIdx+1)' + (e')*edge.information*A)';
    b(edge.toIdx:edge.toIdx+2) = (b(edge.toIdx:edge.toIdx+2)' + (e')*edge.information*B)';

    H(edge.fromIdx:edge.fromIdx+1,edge.fromIdx:edge.fromIdx+1) = H(edge.fromIdx:edge.fromIdx+1,edge.fromIdx:edge.fromIdx+1) + A'*edge.information*A;
    H(edge.fromIdx:edge.fromIdx+1,edge.toIdx:edge.toIdx+2) = H(edge.fromIdx:edge.fromIdx+1,edge.toIdx:edge.toIdx+2) + A'*edge.information*B;
    H(edge.toIdx:edge.toIdx+2,edge.fromIdx:edge.fromIdx+1) = H(edge.toIdx:edge.toIdx+2,edge.fromIdx:edge.fromIdx+1) + B'*edge.information*A;
    H(edge.toIdx:edge.toIdx+2,edge.toIdx:edge.toIdx+2) = H(edge.toIdx:edge.toIdx+2,edge.toIdx:edge.toIdx+2) + B'*edge.information*B;

    if sum(isnan(b))
        disp('b nan');
    end
  end
end

if (needToAddPrior)
  % TODO: add the prior for one pose of this edge
  % This fixes one node to remain at its current location

  H(g.ss_num*2+g.rblm_num*2+1:g.ss_num*2+g.rblm_num*2+3,g.ss_num*2+g.rblm_num*2+1:g.ss_num*2+g.rblm_num*2+3) = eye(3);

  %H(g.ss_num*2+g.rblm_num*2+1:g.ss_num*2+g.rblm_num*2+3,1:g.ss_num*2+g.rblm_num*2) = zeros(3,g.ss_num*2+g.rblm_num*2);

  %H(1:g.ss_num*2+g.rblm_num*2,g.ss_num*2+g.rblm_num*2+1:g.ss_num*2+g.rblm_num*2+3) = zeros(g.ss_num*2+g.rblm_num*2,3);

  %needToAddPrior = false;
end

disp('solving system');

% TODO: solve the linear system, whereas the solution should be stored in dx
% Remember to use the backslash operator instead of inverting H

% dx = H\(-b);

if i==1
  H_diag = diag(H);
  lm_lamda = lm_tau*max(H_diag);
end

%H = H+diag(diag(H))*0.1;
lamda_diag_H = lm_lamda*diag(diag(H));
H = H + lamda_diag_H;


dx = H\(-b);

lm_g_new = g;
lm_g_new.x = lm_g_new.x + dx;
lm_F_new = compute_global_error_2d(lm_g_new);

% !Atention: this way of computing lm_rau is based on H=H+uI, here we used H =
% H+u*diag(H). So, lm_rau can be computed as :
% (lm_F_res-lm_F_new)/((1/2)*(dx')*(diag(diag(H))*lm_lamda*dx-b))
lm_rau = (lm_F_res-lm_F_new)/...
    ((1/2)*(dx')*(lm_lamda*dx-b)); 
% lm_rau = (lm_F_res-lm_F_new)/...
%     ((1/2)*(dx')*(diag(diag(H-lamda_diag_H))*lm_lamda*dx-b)); 

% kinda cheeting
if(lm_F_res-lm_F_new)<0
    lm_rau=-1;
end


% recover H
H = H-lamda_diag_H;
% if (needToAddPrior)
% 
%   H(g.ss_num*2+g.rblm_num*2+1:g.ss_num*2+g.rblm_num*2+3,g.ss_num*2+g.rblm_num*2+1:g.ss_num*2+g.rblm_num*2+3) = ...
%       H(g.ss_num*2+g.rblm_num*2+1:g.ss_num*2+g.rblm_num*2+3,g.ss_num*2+g.rblm_num*2+1:g.ss_num*2+g.rblm_num*2+3)-eye(3);
% end

end
