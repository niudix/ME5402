function []= Theta_plot(Theta,dt)
% 时间向量（假设每个样本代表一个单位时间）
t = (1:size(Theta, 1))*dt;

% 计算速度（差分）
Velocity = diff(Theta, 1, 1);

% 计算加速度（差分的差分）
Acceleration = diff(Velocity, 1, 1);

% 绘制位置图
figure
plot(t, Theta);
title('Joint Positions vs. Time');
xlabel('Time');
ylabel('Position (Radians)');
legend('Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6');


% 绘制速度图
figure
plot(t(1:end-1), Velocity);
title('Joint Velocities vs. Time');
xlabel('Time');
ylabel('Velocity (Radians/sec)');
legend('Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6');


% 绘制加速度图
figure
plot(t(1:end-2), Acceleration);
title('Joint Accelerations vs. Time');
xlabel('Time');
ylabel('Acceleration (Radians/sec^2)');
legend('Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6');


end