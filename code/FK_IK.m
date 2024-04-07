clear;
clc;

%% FK

syms theta1 theta2 theta3 theta4 theta5 theta6
theta_num=[-0.0000    1.1638   -0.2425    0.0658   -0.0000    0.0601];
theta_symb=[theta1, theta2, theta3, theta4, theta5, theta6];

theta=theta_num;
T=FK_func(theta);
disp(T);


%% IK
%roll, pitch, yaw, x_pos, y_pos, z_pos
theta_solved=IK_func(T);

