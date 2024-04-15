# NUS ME5402/EE5106 ADVANCED ROBOTICS



## 0. Tasks Description

1. Conduct an independent study of Universal Robot UR5e as a group (Refer to UR5e Product Fact Sheet, literature, or a view of the physical robot).

2. Model the manipulator using the classical D-H representation of UR5e
3. Derivate the forward kinematics
4. Solve the analytic inverse kinematics problem, which involves finding the set of joint configurations that satisfy the desired position and orientation of the final link.

5. Define an upright 3D plane resembling the back of a human body in your workspace, and identify 8 target points on the plane.
   For reference: the average length of a torso is between 45 to 60 cm; the average shoulder
   width is between 40 to 50 cm; and the waist width ranges from 30 to 40 cm. Make any
   assumptions you see fit.

6. Write a program to move the distal end of the manipulator so as to insert a needle onto each point.
   6.1 Start from the zero position of the manipulator;
   6.2 Pause at the beginning and ending of each needle insertion (piercing);
   6.3 Define the speed at which piercing should be performed. Ensure the piercing is completed at the specified speed. (Hint: Use the inverse Inverse Jacobian method)

   

## 1.  Before Execution

-  MATLAB's Robotics System Toolbox is required for the ur5e simulation. Make sure you have it installed.

- Make sure you change the path in **Trajectory_plan_ur5e.m** line 5 to your own. This is the path for ur5e's appearance description document, which is important for simulation. 

  ```matlab
  addpath 'C:\Users\ndxnd\Documents\MATLAB\Examples\R2023a\urseries\MotionPlanningRBTSimulationUR5eBinPickingManipulatorRRTExample'
  ```

  **Notice**: If you don't know the path. You can comment the line and run **Trajectory_plan_ur5e.m**. The error message in Matlab may tell you the path.

  <img src="/media/error_msg.png" alt="2D_grid_map" style="zoom: 50%;" />



## 2. Forward kinematics and Inverse kinematics

Execute **FK_IK.m** 

- The end positions were determined using forward kinematics based on the given angles of each joint. 
- The IK function was then used to solve the joint angles based on the transformation matrix obtained from the forward kinematics and the resulting angles were compared with the initially given joint angles.

<img src="/media/Fk_IK_result.png" alt="2D_grid_map" style="zoom: 50%;" />



## 3. Trajectory Planning

Execute **Trajectory_plan_ur5e.m** 

<img src="/media/robot_arm.png" alt="2D_grid_map" style="zoom: 33%;" />

The command window shows the status of ur5e

<img src="/media/Trajectory_planning.png" alt="2D_grid_map" style="zoom: 50%;" />
