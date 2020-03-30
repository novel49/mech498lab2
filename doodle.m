% Annabel Chang & Tori Joshi
% MECH 498
% Lab 2: Robot Picasso
% March 30, 2020

function doodle()
% Steps:
% Run doodle() to create doodle.mat
% Run fanucDraw3D('doodle.mat')
% profit

% Set down cone
n = 200; % size factor
m = 800; % shift factor
t = linspace(0,2*pi,2001);
r = sqrt(abs(2*sin(10*t)));
[x y]=pol2cart(t,r);
z2 = 1.5*(x.^2+y.^2);
s = [x*n+m; y*n+m; z2*n];
c = ones(1,length(s));

% Dispense ice cream
n2 = n*1.4;
t2 = linspace(0,2*pi,1001);
x2 = sin(t2).*cos(10*t2);
y2 = sin(t2).*sin(10*t2);
z2 = cos(t2);
s2 = [x2*n2+m; y2*n2+m; z2*n2+800];
s = [s s2];
c = [c 2*ones(1,length(s2))];

% Put the cherry on top
n3 = n*0.5; % fruit
t3 = linspace(0,2*pi,351);
x3 = sin(t3).*cos(10*t3);
y3 = sin(t3).*sin(10*t3);
z3 = cos(t3);
s3 = [x3*n3+m; y3*n3+m; z3*n3+1200];
s = [s s3];
c = [c 3*ones(1,length(s3))];
n4 = n*0.13; % stem
t4 = 1:10;
x4 = zeros(1,length(t4));
y4 = zeros(1,length(t4));
z4 = t4;
s4 = [x4*n4+m; y4*n4+m; z4*n4+1300];
s = [s s4];
c = [c 3*ones(1,length(s4))];

save IceCream.mat c s
% Enjoy

end
