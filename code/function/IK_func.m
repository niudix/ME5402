function[theta_sol]=IK_func(Position)
    % Define symbolic variables
    syms theta1 theta2 theta3 theta4 theta5 theta6
    
    % Define array of symbolic variables
    theta_symb = [theta1, theta2, theta3, theta4, theta5, theta6];
    
    % Get transformation matrix from symbolic FK function
    T_symb = FK_func(theta_symb);
    
    % Convert symbolic transformation matrix to a numerical function
    T_func = matlabFunction(T_symb, 'Vars', {theta_symb});
    
    % Define the error function
    error_function = @(theta_values) compute_error(T_func, theta_values, Position);
    
    % Initial guess
    theta0 = zeros(1, 6); 
    
    % Define joint angle limits
    lb = [-pi, -pi/2, -pi, -pi, -pi/2, -pi]; % Lower bounds
    ub = [pi, pi/2, pi, pi, pi/2, pi]; % Upper bounds
    
    % Call fmincon
    options = optimoptions('fmincon', 'Display', 'off', 'Algorithm', 'sqp');

    [theta_sol, ~, exitflag, ~] = fmincon(error_function, theta0, [], [], [], [], lb, ub, [], options);
    
end

function error = compute_error(T_func, theta_values, Position)
    % Calculate the current transformation matrix using the numerical function
    T_current = T_func(theta_values);
    
    % Target transformation matrix - temporarily replaced by position vector, in practice should be generated based on Position
    % Code for generating target transformation matrix T_target from Position is omitted here
    
    % Compute error
    error = norm(T_current - Position); % Please adjust the error calculation method according to actual situation
end
