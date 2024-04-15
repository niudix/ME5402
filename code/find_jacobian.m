function [jL]=find_jacobian
    syms theta1 theta2 theta3 theta4 theta5 theta6
    theta_symb=[theta1, theta2, theta3, theta4, theta5, theta6];
    T = FK_func(theta_symb);
    %Extract the position vector from the transformation matrix.
    p = T(1:3, 4);
    %Calculate the jacobian matrix of the manipulator 
    %Find the number of elements in the position vector and number of joints
    numVec = length(p); 
    numJoint = length(theta_symb); 
    %Create an array of all zeros. 
    jL = sym(zeros(numVec, numJoint)); 
    for ind1 = 1:numVec
        for ind2 = 1:numJoint
            %Compute the partial derivatives of position vector with
            %respect to each joint angle. 
            jL(ind1, ind2) = diff(p(ind1), theta_symb(ind2));
        end
    end

end
