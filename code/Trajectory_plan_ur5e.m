%% Initialization
clc
clear
close all
addpath 'C:\Users\ndxnd\Documents\MATLAB\Examples\R2023a\urseries\MotionPlanningRBTSimulationUR5eBinPickingManipulatorRRTExample'
ur5eRBT = loadrobot('universalUR5e','DataFormat','row');
ur5e = exampleHelperAddGripper(ur5eRBT);
% Home Position
homePosition =[0     -pi/2     pi/2     0     0     0];


% Set home position of each joint
ur5e.Bodies{1, 3}.Joint.HomePosition = homePosition(1);
ur5e.Bodies{1, 4}.Joint.HomePosition = homePosition(2);
ur5e.Bodies{1, 5}.Joint.HomePosition = homePosition(3);
ur5e.Bodies{1, 6}.Joint.HomePosition = homePosition(4);
ur5e.Bodies{1, 7}.Joint.HomePosition = homePosition(5);
ur5e.Bodies{1, 8}.Joint.HomePosition = homePosition(6);

% Show robot at home position
f1 = figure;
show(ur5e,homePosition,'Frames','on','PreservePlot',false,'Collisions','off','Visuals','on');
hold on

% Limit the Z-axis
zlim([-0.5 1.5]); % 设置Z轴的显示范围从-1到无穷大
xlim([-1 1]); % 设置Z轴的显示范围从-1到无穷大
ylim([-1 1]); % 设置Z轴的显示范围从-1到无穷大

%%
% Final_pos=[0.05,0.8,0.5,-pi/2,0,0];
pos_list=[0.05,0.6,0.58,-pi/2,0,0;
          0.05,0.6,0.55,-pi/2,0,0;
          0.05,0.6,0.52,-pi/2,0,0;
          0.08,0.6,0.52,-pi/2,0,0;
          0.12,0.6,0.52,-pi/2,0,0;
          0.05,0.6,0.4,-pi/2,0,0;
          0.05,0.6,0.35,-pi/2,0,0;
          0.05,0.6,0.3,-pi/2,0,0];
Jac_mtx = find_jacobian;

for j=(1:8)
    Final_pos=pos_list(j,:);
    Final_T=transform_mtx(Final_pos);
    Final_Theta = IK_func(Final_T);
    if j~=1
        First_Theta_T=transform_mtx(pos_list(j-1,:));
        homePosition=IK_func(First_Theta_T);
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
    a0 = homePosition;
    a1 = theta_dot_0;
    a2 = theta_ddot_0 / 2;
    a3 = (20*(Final_Theta - homePosition) - (8*theta_dot_f + 12*theta_dot_0)*t_f ...
        - (3*theta_ddot_0 - theta_ddot_f)*t_f^2) / (2*t_f^3);
    a4 = (30*(homePosition - Final_Theta) + (14*theta_dot_f + 16*theta_dot_0)*t_f ...
        + (3*theta_ddot_0 - 2*theta_ddot_f)*t_f^2) / (2*t_f^4);
    a5 = (12*(Final_Theta - homePosition) - 6*(theta_dot_f + theta_dot_0)*t_f ...
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
    
    Total_inser_time=5;
    Total_insert_len=0.06;
    Total_insert_step=30;
    Inser_Theta=zeros(Total_insert_step,6);
    Inser_Theta(1,:)=Theta(step,:);
    dt=Total_inser_time/Total_insert_step;
    Insert_Start_point=Final_pos;
    for i=1:Total_insert_step-1
         Insert_End_point=Insert_Start_point+[0,Total_insert_len/Total_insert_step,0,0,0,0];
         Inser_Theta(i+1,:)=Insert(Insert_Start_point,Insert_End_point,Inser_Theta(i,:),dt,Jac_mtx);
         Insert_Start_point=Insert_End_point;
    end

    rateObj = rateControl(20);
    rateinst= rateControl(1/dt);

    for i = 1 : size(Theta)
        show(ur5e,Theta(i,:),'PreservePlot',false,'Frames','off','Collisions','off','Visuals','on','FastUpdate',true);
        
        drawnow
        waitfor(rateObj);
    end
    
    for i = 1 : size(Inser_Theta)
        show(ur5e,Inser_Theta(i,:),'PreservePlot',false,'Frames','off','Collisions','off','Visuals','on','FastUpdate',true);
        drawnow
        waitfor(rateinst);
    end
    for i = size(Inser_Theta) :-1: 1
        show(ur5e,Inser_Theta(i,:),'PreservePlot',false,'Frames','off','Collisions','off','Visuals','on','FastUpdate',true);
        drawnow
        waitfor(rateinst);
    end
end

