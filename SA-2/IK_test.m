clear all;close all;clc;
% TEST example
% This scripts is an example of how your code will be tested. sim_robot plot
% the robot in the 3D space given the DH parameters and the joint
% configuration. You can use the simulation for visual inspection purposes
% in both questions.

% DH parameters format based on Siciliano's book:
%
%      | a_1 | alpha_1 | d_1 | theta_1 |
% DH = | ... | ....... | ... | ....... | 
%      | a_n | alpha_n | d_n | theta_n |
%
% with alpha_i and theta_i in radiant
% EXAMPLE 1: Three-link Planar Arm
q = rand(3,1);
jtype = [0;0;0];
DH(:,1) = rand(3,1);            % a  
DH(:,2) = rand(3,1);           % alpha
DH(:,3) = zeros(3,1);           % d
DH(:,4) = q;                    % theta

q_des = rand(3,1);              % desired joint configuration
[T, J1] = FK(DH, jtype, q_des);
p_des = T(1:3,4);               % desired end-effector configuration
R = T(1:3,1:3);
phi_des = atan2(R(2,1),R(1,1));

theta_e = atan2(-R(3,1), sqrt(R(3,2)^2+R(3,3)^2)) ;
psi_e = atan2(R(3,2), R(3,3)) ;
o_e = [theta_e;psi_e;phi_des];

Q = IK(DH, jtype, q, p_des, o_e); 

% figure();
% plot(transpose(Q))
% xlabel('num iter')
% ylabel('q')
% legend('q_1','q_2','q_3');

[T] = FK(DH, jtype, Q(:,end));

pfinal = T(1:3,4);
R = T(1:3,1:3);
phifinal = atan2(R(2,1),R(1,1));

% Check the final error between the current end-effector pose and the desired one
p_error = p_des-pfinal;
o_error = phi_des-phifinal;%mod(phi_des-phifinal, 2*pi);
 
fprintf("p_des %2f \n" ,p_des)
fprintf("pfinal %2f \n" ,pfinal)
fprintf("p_error %2f \n" ,p_error)
fprintf("\n")
fprintf("phi_des %2f \n" ,phi_des)
fprintf("phifinal %2f \n" ,phifinal)
fprintf("o_error %2f \n" ,o_error)

% Simulate the robot at each step of the iterative inverse kinematics 
% and observe how the end-effector reaches the desired pose
figure();
for steps = 1:size(Q,2)
    sim_robot(DH,Q(:,steps),jtype)
end