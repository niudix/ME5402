function T = transform(position)
    % The 'position' input is a vector that includes the xyz coordinates
    % and the roll, pitch, yaw angles
    % Example format of 'position': [x, y, z, roll, pitch, yaw]
    
    % Extract the position coordinates and Euler angles
    x = position(1);
    y = position(2);
    z = position(3);
    roll = position(4);
    pitch = position(5);
    yaw = position(6);

    % Compute the rotation matrix based on roll, pitch, yaw
    Rz = [cos(yaw), -sin(yaw), 0;
          sin(yaw), cos(yaw), 0;
          0, 0, 1];

    Ry = [cos(pitch), 0, sin(pitch);
          0, 1, 0;
          -sin(pitch), 0, cos(pitch)];

    Rx = [1, 0, 0;
          0, cos(roll), -sin(roll);
          0, sin(roll), cos(roll)];

    % Combine to get the overall rotation matrix
    R = Rz * Ry * Rx;

    % Construct the homogeneous transformation matrix T
    T = [R, [x; y; z]; 0, 0, 0, 1];
end
