function [q] = Insert(Start_point,End_point,init_q,dt,jac_mtx)
    
    % 计算位移向量
    displacement = (End_point(:,1:3)-Start_point(:,1:3))/dt;

    % 计算雅可比矩阵
    Jac_val = find_jacobian_num(jac_mtx,init_q);
    
    end_vel=displacement';
    
    % 逆向运动学计算dq
    dq = pinv(Jac_val) * end_vel; % 仅考虑位置，不考虑姿态
    
    % 更新关节角度
    q = init_q + dq'*dt;
    
end

