function End_Point_plot(Theta, dt, Jac_mtx)
    % Theta is a 6*n matrix
    % dt is the time interval between two consecutive angle recordings
    % Jac_mtx is the symbolic form of the Jacobian matrix

    n = size(Theta, 1);  % Get the number of angles
    End_Positions = zeros(3, n);  % Initialize the end position storage matrix
    End_Velocities = zeros(3, n-1);  % Initialize the end velocity storage matrix
    
    % Traverse all time points to calculate end positions and velocities
    for i = 1:n
        % Calculate current position
        T = FK_func(Theta(i, :));  % Call the forward kinematics function
        End_Positions(:, i) = T(1:3, 4);  % Store position information
        
        if i < n
            % Calculate velocity
            Jac_val = find_jacobian_num(Jac_mtx, Theta(i, :));  % Convert Jacobian matrix to numeric form
            theta_dot = (Theta(i+1, :) - Theta(i, :)) / dt;  % Calculate joint angular velocity
            End_Velocities(:, i) = Jac_val * theta_dot';  % Calculate end velocity
        end
    end

    % Plot position
    figure;
    subplot(2, 1, 1);
    plot(End_Positions');
    title('End Effector Position');
    xlabel('Time step');
    ylabel('Position (m)');
    legend('X', 'Y', 'Z');
    
    % Plot velocity
    subplot(2, 1, 2);
    plot(End_Velocities');
    title('End Effector Velocity');
    xlabel('Time step');
    ylabel('Velocity (m/s)');
    legend('X', 'Y', 'Z');
    
    sgtitle('End Effector Dynamics');
end
