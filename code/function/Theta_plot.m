function []= Theta_plot(Theta,dt)
% Time vector (assuming each sample represents one unit of time)
t = (1:size(Theta, 1))*dt;

% Compute velocity (difference)
Velocity = diff(Theta, 1, 1);

% Compute acceleration (difference of the difference)
Acceleration = diff(Velocity, 1, 1);

% Plot position graph
figure
plot(t, Theta);
title('Joint Positions vs. Time');
xlabel('Time');
ylabel('Position (Radians)');
legend('Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6');


% Plot velocity graph
figure
plot(t(1:end-1), Velocity);
title('Joint Velocities vs. Time');
xlabel('Time');
ylabel('Velocity (Radians/sec)');
legend('Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6');


% Plot acceleration graph
figure
plot(t(1:end-2), Acceleration);
title('Joint Accelerations vs. Time');
xlabel('Time');
ylabel('Acceleration (Radians/sec^2)');
legend('Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6');


end
