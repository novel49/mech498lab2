% Annabel Chang & Tori Joshi
% MECH 498
% Lab 2: Robot Picasso
% March 27, 2020


function [is_solution,joint_angles] = fanucIK(T,prev_joint_angles,fanuc)
 R = T(1:3,1:3);
 Rnew = R*[0;0;-180];
 
 theta1 = atan2(Rnew(2,1),Rnew(1,1));

end