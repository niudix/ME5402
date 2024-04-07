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

%定义轨迹规划初始关节角度（First_Theta）和终止关节角度（Final_Theta）,步数100
First_Theta = [0     -pi/2     pi/2     0     0     0];
Final_Theta = [-pi/2    -pi/3    -pi/4    pi/3    pi/5   pi/6];
step = 100;
[q,qd,qdd] = jtraj(First_Theta,Final_Theta,step);
robot.plot(q)

figure(2);
subplot(2,2,1);
i = 1:6;
plot(q(:,i));
grid on;
title('关节位置');

subplot(2,2,2);
i = 1:6;
plot(qd(:,i));
grid on;
title('关节速度');

subplot(2,2,3);
i = 1:6;
plot(qdd(:,i));
grid on;
title('关节加速度');
