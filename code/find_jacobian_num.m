function [J_val]=find_jacobian_num(jL,joint_ang)
    syms theta1 theta2 theta3 theta4 theta5 theta6
    theta1_val=joint_ang(1);
    theta2_val=joint_ang(2);
    theta3_val=joint_ang(3);
    theta4_val=joint_ang(4);
    theta5_val=joint_ang(5);
    theta6_val=joint_ang(6);
    J_val=subs(jL, [theta1,theta2,theta3,theta4,theta5,theta6], [theta1_val, theta2_val, theta3_val, theta4_val, theta5_val, theta6_val]);
    J_val=double(J_val);
end