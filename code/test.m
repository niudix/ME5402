clear;
clc;

% 建立机器人模型
L1 = Link([0 0.1625 0 pi/2 0]); % 定义连杆的D-H参数
L2 = Link([pi/2 0 -0.425 0 0]);
L3 = Link([0 0 -0.3922 0 0]);
L4 = Link([0 0.1333 0 pi/2 0]);
L5 = Link([pi 0.0997 0 -pi/2 0]);
L6 = Link([0 0.0996 0 0 0]);
robot = SerialLink([L1 L2 L3 L4 L5 L6], 'name', 'UR5e');
q_min = [0 -pi/2 -pi/2 -pi/2 -pi/2 -pi/2];
q_max = [pi pi/2 pi/2 pi/2 pi/2 pi/2];

% 定义插入点
points = [0.3 0.3 0.3;
          0.2 0.3 0.2;
          0.1 0.3 0.1;
          0.2 0.3 0.2;
          -0.2 0.3 0.2;
          -0.3 0.3 -0.3;
          -0.1 0.3 -0.1;
          0.1 0.3 -0.1];

% 轨迹规划步数
step = 100;


% 生成插入八个点的轨迹
for i = 1:size(points, 1) - 1
    % 逆运动学求解
    T1=transl(points(i,:));
    T2=transl(points(i+1,:));
    q_sol = robot.ikine(T1, 'q0', zeros(1, robot.n), 'mask', [1 1 1 0 0 0],'qlim', [q_min; q_max]);
    q_sol2 = robot.ikine(T2, 'q0', zeros(1, robot.n), 'mask', [1 1 1 0 0 0],'qlim', [q_min; q_max]);
    
    %插入点
    %insert()

    %计算轨迹
    [q, qd, qdd] = jtraj(q_sol, q_sol2, step);
    if i == 1
        %插入点
        %insert
        q_traj = q;
    else
        q_traj = [q_traj; q]; % 避免重复插入点
    end
end

% 画出机器人轨迹
robot.plot(q_traj);
