function [q] = Insert(Start_point,End_point,init_q,dt,jac_mtx)
    
    % Calculate the displacement vector
    displacement = (End_point(:,1:3)-Start_point(:,1:3))/dt;

    % Calculate the numerical Jacobian matrix
    Jac_val = find_jacobian_num(jac_mtx,init_q);
    
    end_vel=displacement';
    
    % Calculate dq using inverse kinematics
    dq = pinv(Jac_val) * end_vel; % Only considering position, not orientation
    
    % Update joint angles
    q = init_q + dq'*dt;
    
end
