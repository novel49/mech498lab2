% Annabel Chang & Tori Joshi
% MECH 498
% Lab 2: Robot Picasso
% March 30, 2020


function [is_solution,joint_angles] = fanucIK(T,prev_joint_angles,fanuc)
% Pull out rotation matrix
% Account for fram 6 being offset from frames 4 & 5 (bring the end effector
% back to the same origin for {4}, {5}, and {6} and determine the new
% position vector.
R = T(1:3,1:3);
P = T(1:3, 4);
Rnew = R * [0;0;-fanuc.parameters.l_6];
Pprime = P + Rnew;

%THETA1
%Determine theta1 (use x & y coordinates to determine rotation)
theta1_1 = atan2(Pprime(2,1),Pprime(1,1));
%Theta1 should be between -5pi/6 and +5pi/6.
if theta1_1 < 0
    theta1_2 = theta1_1 + pi;
else
    theta1_2 = theta1_1 - pi;
end

%Verify which solution for theta1 is closer to the previous joints. And the
%knowledge of what case you are in is a handy tool that will help us later.
if (abs(theta1_1 - prev_joint_angles(1)) <= abs(theta1_2 - prev_joint_angles(1)))
    theta1 = theta1_1;
    plus180 = 0;
else
    theta1 = theta1_2;
    plus180 = 1;
end

%THETA3
%Find hypotenuse between links 4 and 5 using Pythagorean Theorem (called r)
r = sqrt(fanuc.parameters.l_4^2 + fanuc.parameters.l_5^2);
%We can find angle made by these offsets using invrese tangent of the triangle made by the
%offsets (called gamma)
gamma = atan2(fanuc.parameters.l_4,fanuc.parameters.l_5);
%find the length between {2} and {4}
if plus180 == 0
    s = sqrt(Pprime(2,1)^2 + Pprime(1,1)^2) - fanuc.parameters.l_2;
else
    s = -(sqrt(Pprime(2,1)^2 + Pprime(1,1)^2) + fanuc.parameters.l_2);
end
t = sqrt(s^2 + Pprime(3,1)^2);
%theta3prime - use right triangle to find s, use that triangle to find
%theta3prime. Also, here you need to know whether you used the alternate
%theta1 term
cos3 = round(((t^2 - r^2 - fanuc.parameters.l_3^2)/(2*r*fanuc.parameters.l_3)),10);
%Take both cases into account.
theta3prime_minus = atan2(-sqrt(1-cos3^2),cos3);
theta3prime_plus = atan2(sqrt(1-cos3^2),cos3);

%From these, calculate theta3
theta3minus = theta3prime_minus + pi/2 - gamma;
theta3plus = theta3prime_plus + pi/2 - gamma;

%Choose the theta3 value that is closest to the previous joint.
if (abs(theta3minus - prev_joint_angles(3)) <= abs(theta3plus - prev_joint_angles(3)))
    theta3 = theta3minus;
else
    theta3 = theta3plus;
end

%THETA2
%Determine the length between {2} and the base of {4} on the XY plane
a = s;
b = Pprime(3,1);
%Determine the distance between {2} and {4}
c = sqrt(a^2 + b^2);
%Determine angle between ground and {4} - Law of Cosines
phi = acos((b^2 - c^2 - a^2)/(-2*a*c));
%Determine angle between c and Link 3 - Law of Cosines
psi = acos((r^2 - c^2 - fanuc.parameters.l_3^2)/(-2*c*fanuc.parameters.l_3));
%Subtract these angles from 90 to find the desired theta2 angle
theta2 = (-1)^plus180*psi + phi - pi/2;

%END EFFECTOR
%Isolate the numerical transformation matrix for the end effector to compare numerical and symbolic elements while saving computational power.
T1 = dhtf(0,0,0,theta1);
T2 = dhtf(pi/2,fanuc.parameters.l_2,0,theta2+pi/2);
T3 = dhtf(0,fanuc.parameters.l_3,0,theta3);
T13 = T1 * T2 * T3;

R13 = T13(1:3,1:3)';
P13 = -R13 * T13(1:3,4);
invT13 = [R13 P13; 0 0 0 1];
T46 = invT13*T;

%THETA5
%Using the symbolic solution for T46, it is possible to isolate sin and cos
%values for each angle, and use them to find the numerical theta values.
sin5  = sqrt(T46(2,1)^2+T46(2,2)^2);
theta5_1 = atan2(sin5,-T46(2,3));
theta5_2 = atan2(-sin5,-T46(2,3));

%Because there are two options for theta5, choose the one closest to the
%current position.
if (abs(theta5_1 - prev_joint_angles(5)) <= abs(theta5_2 - prev_joint_angles(5)))
    theta5 = theta5_1;
else
    theta5 = theta5_2;
end

%Heads up, there is a singularity when theta5 = 0.

%THETA4
cos4 = T46(1,3)/sin(theta5);
sin4 = T46(3,3)/sin(theta5);
theta4 = atan2(sin4,cos4);

%THETA6
cos6 = T46(2,1)/sin(theta5);
sin6 = T46(2,2)/-sin(theta5);
theta6 = atan2(sin6,cos6);

%Throw all the calculated angles into one vector.
joint_angles = [theta1 theta2 theta3 theta4 theta5 theta6];

%Verify that the solution actually works with the robot in physical
%space...
if fanuc.workspace(1) <= T(1,4) <= fanuc.workspace(2)...
        && fanuc.workspace(3) <= T(2,4) <= fanuc.workspace(4)...
        && fanuc.workspace(5) <= T(3,4) <= fanuc.workspace(6)...
        && fanuc.joint_limits{1}(1) <= joint_angles(1) <= fanuc.joint_limits{1}(2)...
        && fanuc.joint_limits{2}(1) <= joint_angles(2) <= fanuc.joint_limits{2}(2)...
        && fanuc.joint_limits{3}(1) <= joint_angles(3) <= fanuc.joint_limits{3}(2)...
        && fanuc.joint_limits{4}(1) <= joint_angles(4) <= fanuc.joint_limits{4}(2)...
        && fanuc.joint_limits{5}(1) <= joint_angles(5) <= fanuc.joint_limits{5}(2)...
        && fanuc.joint_limits{6}(1) <= joint_angles(6) <= fanuc.joint_limits{6}(2)
    is_solution = 1;
else
    is_solution = 0;
end

