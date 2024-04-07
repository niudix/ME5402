function[Theta]=Linear_Traj(init_pos ,end_pos)


% 使用原机械臂的后四个关节的D-H参数来定义新机械臂模型的连杆
L1_new = Link([0       0        -0.3922    0         0]); % 原L3
L2_new = Link([0       0.1333   0          pi/2      0]); % 原L4
L3_new = Link([pi      0.0997   0          -pi/2     0]); % 原L5
L4_new = Link([0       0.0996   0          0         0]); % 原L6

% 创建新的SerialLink机械臂模型
robot_new = SerialLink([L1_new L2_new L3_new L4_new], 'name', 'Modified UR5e');

% 定义移动的总时间T和步数step
T = 5; % 总时间，假设为5秒
step = 30; % 步数

% 生成直线路径上的点
t = linspace(0, 1, step); % 归一化时间向量
path_points = zeros(step, 6); % 初始化路径点数组

for i = 1:step
    for j = 1:6
        path_points(i,j) = init_pos(j) + t(i) * (end_pos(j) - init_pos(j));
    end
end

% 计算路径上每个点的关节角度
Theta = zeros(step, 6); % 初始化关节角度数组

for i = 1:step
    T = transl(path_points(i,1:3)) * rpy2tr(path_points(i,4:6)); % 计算目标位置的变换矩阵
    theta_i = robot_new.ikine(T, 'mask', [1 1 1 0 0 0]); % 用于四自由度的机械臂
    Theta(i,:) = theta_i; % 存储关节角度
end

