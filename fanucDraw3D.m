% Annabel Chang & Tori Joshi
% MECH 498
% Lab 2: Robot Picasso
% March 30, 2020

function fanucDraw3D(path_file)
% MECH 498/598 - Intro to Robotics - Spring 2016
% Lab 2 - Inverse Kinematics
%
%    DESCRIPTION - Plot a graphical representation of the FANUC S-500
%    Industrial robot with attached coordinate frames as it moves through a
%    series of poses defined by path_file.
%
%    ADDITIONAL CODE NEEDED: lots

% Initialize the fanuc struct
fanuc = fanucInit();

% Get path position and color data
data = load(path_file);
s = data.s; % position
c = data.c; % color

% Draw FANUC initially in zero position (do not change)
prev_angles = zeros(1,6);
fanuc.handles = drawFanuc(prev_angles,fanuc);
hold on;

% Draw in 3D
for t = 1:size(s,2)
    
    % Set desired brush color from path file (think about how to handle
    % changes in color)
    fanuc.brush = data.c(t);
    
    % Select desired orientation for the tool (your choice)
    % Make the tool point downward
    tool_rotate = [1 0 0; 0 -1 0; 0 0 -1];
    
    % Set desired position for the tool from path file (not your choice)
    final_point = [data.s(1,t); data.s(2,t); data.s(3,t)];
    final_frame = final_point - [0; 0; fanuc.parameters.l_t];
    T = [tool_rotate final_frame];
    T = [T; 0 0 0 1];
        
% Solve inverse kinematics for nearest solution
%[T,~] = fanucFK(joint_angles,fanuc);

% R = T(1:3,1:3);
% P = T(1:3, 4);
% Rnew = R * [0;0;fanuc.parameters.l_t];
% Pactual = P + Rnew;
% T(1:3,4) = Pactual;

[is_solution,joint_angles] = fanucIK(T,prev_angles,fanuc);

% Move robot using setFanuc() if solution exists
if is_solution == 1
    setFanuc(joint_angles, fanuc );
    
    % Plot a point at the tool brush tip with the appropriate color
    % (unless the brush selection is zero)
    if fanuc.brush ~= 0     
        % Update previous joint angles
        prev_angles = joint_angles;
    end
end
end

