function[theta_sol]=IK_func(Position)
    % 定义符号变量
    syms theta1 theta2 theta3 theta4 theta5 theta6
    
    % 定义符号变量数组
    theta_symb = [theta1, theta2, theta3, theta4, theta5, theta6];
    
    % 从符号FK函数获取转换矩阵
    T_symb = FK_func(theta_symb);
    
    % 将符号转换矩阵转换为数值函数
    T_func = matlabFunction(T_symb, 'Vars', {theta_symb});
    
    % 定义误差函数
    error_function = @(theta_values) compute_error(T_func, theta_values, Position);
    
    % 初始猜测
    theta0 = zeros(1, 6); 
    
    % 定义关节角度界限
    lb = [-pi, -pi/2, -pi, -pi, -pi/2, -pi]; % 下界
    ub = [pi, pi/2, pi, pi, pi/2, pi]; % 上界
    
    % 调用fmincon
    options = optimoptions('fmincon', 'Display', 'iter', 'Algorithm', 'sqp');
    [theta_sol, ~, exitflag, ~] = fmincon(error_function, theta0, [], [], [], [], lb, ub, [], options);
    
    % 显示结果
    if exitflag > 0
        disp('解 theta:');
        disp(theta_sol);
    else
        disp('求解失败。');
    end
end

function error = compute_error(T_func, theta_values, Position)
    % 使用数值函数计算当前的转换矩阵
    T_current = T_func(theta_values);
    
    % 目标转换矩阵-暂时用位置向量代替，实际中应根据Position生成
    % 这里省略了从Position生成目标转换矩阵T_target的代码
    
    % 计算误差
    error = norm(T_current - Position); % 请根据实际情况调整误差计算方法
end
