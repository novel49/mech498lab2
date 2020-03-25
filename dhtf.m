% Annabel Chang & Tori Joshi
% MECH 498
% Lab 2: Robot Picasso
% March 27, 2020

function T = dhtf(alpha, a, d, theta)
%Given the DH parameters, find the corresponding standard transformation
%matrix for each row.

T = [cos(theta) -sin(theta) 0 a;
    sin(theta)*cos(alpha) cos(theta)*cos(alpha) -sin(alpha) -sin(alpha)*d;
    sin(theta)*sin(alpha) cos(theta)*sin(alpha) cos(alpha) cos(alpha)*d;
    0 0 0 1];
end