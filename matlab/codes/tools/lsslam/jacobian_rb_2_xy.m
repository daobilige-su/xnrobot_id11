function J = jacobian_rb_2_xy(rb)

% xy = rb(1)*[cos(rb(2));sin(rb(2))];

J = [cos(rb(2)),-rb(1)*sin(rb(2));...
     sin(rb(2)),rb(1)*cos(rb(2))];

end