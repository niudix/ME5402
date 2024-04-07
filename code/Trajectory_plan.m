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
Final_pos=[0.05,0.8,0.5,-pi/2,0,0];
pos_list=[0.05,0.8,0.5,-pi/2,0,0;
          -0.05,0.8,0.5,-pi/2,0,0;
          -0.12,0.77,0.42,-pi/2,0,0;
          -0.12,0.75,0.32,-pi/2,0,0;
          -0.05,0.8,0.25,-pi/2,0,0;
          0.05,0.8,0.25,-pi/2,0,0;
          -0.12,0.75,0.32,-pi/2,0,0;
          0.12,0.77,0.42,-pi/2,0,0;];

for j=(1:8)
    Final_pos=pos_list(j,:);
    Final_T=transform_mtx(Final_pos);
    Final_Theta = IK_func(Final_T);
    if j~=1
        First_Theta_T=transform_mtx(pos_list(j-1,:));
        First_Theta=IK_func(First_Theta_T);
    end
    step = 100;
    
    
    % 时间参数
    T = 1; % 假设从起点到终点的时间为1秒
    t = linspace(0, T, step); % 生成时间向量
    
    % 假设初始速度和加速度都为零，终止速度和加速度也为零
    theta_dot_0 = zeros(1, 6); % 初始速度
    theta_ddot_0 = zeros(1, 6); % 初始加速度
    theta_dot_f = zeros(1, 6); % 终止速度
    theta_ddot_f = zeros(1, 6); % 终止加速度
    t_f = 1; % 插值总时间
    
    % 计算五次多项式的系数
    a0 = First_Theta;
    a1 = theta_dot_0;
    a2 = theta_ddot_0 / 2;
    a3 = (20*(Final_Theta - First_Theta) - (8*theta_dot_f + 12*theta_dot_0)*t_f ...
        - (3*theta_ddot_0 - theta_ddot_f)*t_f^2) / (2*t_f^3);
    a4 = (30*(First_Theta - Final_Theta) + (14*theta_dot_f + 16*theta_dot_0)*t_f ...
        + (3*theta_ddot_0 - 2*theta_ddot_f)*t_f^2) / (2*t_f^4);
    a5 = (12*(Final_Theta - First_Theta) - 6*(theta_dot_f + theta_dot_0)*t_f ...
        - (theta_ddot_0 - theta_ddot_f)*t_f^2) / (2*t_f^5);
    
    % 使用五次多项式计算每个时间点的关节角度
    Theta = zeros(step, 6); % 初始化关节角度矩阵
    for i = 1:step
        t_i = t(i);
        Theta(i,:) = a0 + a1*t_i + a2*t_i^2 + a3*t_i^3 + a4*t_i^4 + a5*t_i^5;
    end
    
    % Linear_init=Final_pos;
    % Linear_end=Final_pos+[0,0.01,0,0,0,0];
    % Theta_linear=Linear_Traj(Linear_init,Linear_end);
    % Theta=[Theta;Theta_linear];
    
    % 可视化轨迹
    for i = 1:step
        robot.plot(Theta(i,:));
        drawnow
    end
end
