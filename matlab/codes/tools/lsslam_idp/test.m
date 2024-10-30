syms e_x1 e_y1 e_z1 e_qr1 e_qx1 e_qy1 e_qz1 e_x2 e_y2 e_z2 e_qr2 e_qx2 e_qy2 e_qz2 m_x1 m_y1 m_z1 m_yaw1 m_pitch1 m_roll1 real

x1 = [e_x1;e_y1;e_z1;e_qr1;e_qx1;e_qy1;e_qz1];
x2 = [e_x2;e_y2;e_z2;e_qr2;e_qx2;e_qy2;e_qz2];

z = [m_x1;m_y1;m_z1;m_yaw1;m_pitch1;m_roll1];

M1 = pose3d_quat_to_matrix(x1);
M2 = pose3d_quat_to_matrix(x2);

Mz = transform_matrix_from_trans_ypr(z);

%e = t2v(inv(v2t(z))*(inv(v2t(x1)))*v2t(x2));
e = transform_matrix_to_pose_trans_ypr([Mz(1:3,1:3)',-Mz(1:3,1:3)'*Mz(1:3,4);0,0,0,1]*...
    [M1(1:3,1:3)',-M1(1:3,1:3)'*M1(1:3,4);0,0,0,1]*M2);

A = jacobian(e,[e_x1,e_y1,e_z1,e_qr1,e_qx1,e_qy1,e_qz1]);

