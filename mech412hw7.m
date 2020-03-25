% Tori Joshi
% MECH 412
% HW 7
% 31 March 2020

function mech412hw7part1()

%Clear out your workspace
close all;
clear all;
clc;

%Establish time span, experimentally determined.
%Due to the nature of this code, we start from 0 for both time scales, so I
%will put the ending times in this vector and call it to create the time
%scale.
%I know it's jank, and I'm sorry.
tspan = [15 1]; %in seconds

%Establish numerical values of Epsilon variables, given.
E = [0.5 -5];

for i = 1:numel(E)
    %Create a function handle that contains a vector for the equations for x.. and x.
    odefunc = @(t,x)[x(2); -0.5*x(2)-(4*x(1)*(1+E(i)*x(1)^2))+5*cos(3*t)];
    
    %Set a vector for the initial Conditions for the ODEs (given).
    X0 = [1 -5];
    
    %Use ode45 to solve the array of differential equations
    [t,X] = ode45(odefunc,[0 tspan(i)],X0);
    
    %Plot
    figure()
    plot(t,X(:,1))
    %Label everything correctly
    xlabel('Time (seconds)')
    ylabel('Position')
    title(strcat('Duffer Oscillation Over Time (\epsilon=',num2str(E(i)),')'))
    %Set grid
    grid on
    hold off
end
end