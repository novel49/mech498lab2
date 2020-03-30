% Annabel Chang & Tori Joshi
% MECH 498
% Lab 2: Robot Picasso
% March 30, 2020

function MagicSphere()

%This is a mess and I'm sorry. I'm still experimenting with shapes.

% Steps:
% Create 3D function
% save x y z coordinated to mat file
% Create corresponding color tags
% save to mat file
% Call mat file with robot
% profit

%Create one color spiral.

t = 0:pi/500:pi;
one(1,:) = sin(t).*cos(10*t);
one(2,:) = sin(t).*sin(10*t);
one(3,:) = cos(t);

%Create a second color spiral.

two(1,:) = sin(t).*cos(15*t);
two(2,:) = sin(t).*sin(15*t);
two(3,:) = cos(t);

%Create a third color spiral.

three(1,:) = sin(t).*cos(20*t);
three(2,:) = sin(t).*sin(20*t);
three(3,:) = cos(t);

%Create a 

%Combine them all tpgether (flip the order of the second matrix so the
%robot's path is not jerky between spirals.)

s = [one flip(two,2) three]*100 + [200; 0; 1500];

%Choose colors for different paths.

c = [ones(1,501) ones(1,501)*3 ones(1,501)*4];

save MagicSphere.mat s c

% https://www.mathworks.com/help/matlab/ref/plot3.html
% For more code ideas.

end