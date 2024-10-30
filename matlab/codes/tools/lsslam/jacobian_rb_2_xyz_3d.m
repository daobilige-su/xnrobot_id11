function J = jacobian_rb_2_xyz_3d(rb)

% xy = rb(1)*[cos(rb(3))*cos(rb(2));cos(rb(3))*sin(rb(2));sin(rb(3))];

J = [cos(rb(3))*cos(rb(2)), -rb(1)*cos(rb(3))*sin(rb(2)),   -rb(1)*sin(rb(3))*cos(rb(2));...
     cos(rb(3))*sin(rb(2)), rb(1)*cos(rb(3))*cos(rb(2)),    -rb(1)*sin(rb(3))*sin(rb(2));...
     sin(rb(3)),            0,                              rb(1)*cos(rb(3))];

end