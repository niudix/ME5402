clear;
clc;
%建立机器人模型
%       theta    d        a        alpha     offset
L1=Link([0       0.1625   0        pi/2      0     ]); %定义连杆的D-H参数
L2=Link([pi/2    0        -0.425     0         0     ]);
L3=Link([0       0        -0.3922    0     0     ]);
L4=Link([0       0.1333   0        pi/2      0     ]);
L5=Link([pi      0.0997   0        -pi/2      0     ]);
L6=Link([0       0.0996   0        0         0     ]);
robot=SerialLink([L1 L2 L3 L4 L5 L6],'name','UR5e'); 
 theta=[0     -pi/2     pi/2     0     0     0];%指定的关节角
%theta=[1.0472   -0.8378   -0.4033    0.6398   -0.0000   -0.9696];
p=robot.fkine(theta)%fkine正解函数，根据我们给定的关节角theta，求解出末端位姿p
q=robot.ikine(p)%ikine逆解函数，根据我们给定的末端位姿p，求解出关节角q

teach(robot);
