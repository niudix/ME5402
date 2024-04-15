function End_Point_plot(Theta, dt, Jac_mtx)
    % Theta 是 6*n 的矩阵
    % dt 是相邻两个角度记录之间的时间间隔
    % Jac_mtx 是符号形式的雅可比矩阵

    n = size(Theta, 1);  % 获取角度个数
    End_Positions = zeros(3, n);  % 初始化末端位置存储矩阵
    End_Velocities = zeros(3, n-1);  % 初始化末端速度存储矩阵
    
    % 遍历所有时间点计算末端位置和速度
    for i = 1:n
        % 计算当前位置
        T = FK_func(Theta(i, :));  % 调用正向运动学函数
        End_Positions(:, i) = T(1:3, 4);  % 存储位置信息
        
        if i < n
            % 计算速度
            Jac_val = find_jacobian_num(Jac_mtx, Theta(i, :));  % 转换雅可比矩阵为数值形式
            theta_dot = (Theta(i+1, :) - Theta(i, :)) / dt;  % 计算关节角速度
            End_Velocities(:, i) = Jac_val * theta_dot';  % 计算末端速度
        end
    end

    % 绘制位置
    figure;
    subplot(2, 1, 1);
    plot(End_Positions');
    title('End Effector Position');
    xlabel('Time step');
    ylabel('Position (m)');
    legend('X', 'Y', 'Z');
    
    % 绘制速度
    subplot(2, 1, 2);
    plot(End_Velocities');
    title('End Effector Velocity');
    xlabel('Time step');
    ylabel('Velocity (m/s)');
    legend('X', 'Y', 'Z');
    
    sgtitle('End Effector Dynamics');
end