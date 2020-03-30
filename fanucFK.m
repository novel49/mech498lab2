% Annabel Chang & Tori Joshi
% MECH 498
% Lab 2: Robot Picasso
% March 30, 2020

function [T, fanuc_T] = fanucFK(joint_angles, fanuc)

% Accepts a 6-element vector of joint angles and the structure output
% by the function fanucInit() and returns both the full forward
% kinematics transform matrix T and the cell array of transforms
% fanuc_T of the  form {0T1 , 1T2, 2T3, 3T4, 4T5 , 5T6}

%Create Individual Transformation Matrices
T1 = dhtf(0,0,0,joint_angles(1));
T2 = dhtf(pi/2,fanuc.parameters.l_2,0,joint_angles(2)+pi/2);
T3 = dhtf(0,fanuc.parameters.l_3,0,joint_angles(3));
T4 = dhtf(pi/2,fanuc.parameters.l_4,fanuc.parameters.l_5,joint_angles(4));
T5 = dhtf(-pi/2,0,0,joint_angles(5));
T6 = dhtf(pi/2,0,fanuc.parameters.l_6,joint_angles(6));

%Create final Transformation Matrix
T = T1*T2*T3*T4*T5*T6;
%Create Transformation Cell Array
fanuc_T = {T1,T2,T3,T4,T5,T6};

end