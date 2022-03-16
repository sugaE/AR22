%% TEST example
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


%% EXAMPLE 1: Three-link Planar Arm %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all;close all;clc; 
 
q = rand(3,1);
jtype = [0;0;0];
DH(:,1) = rand(3,1);            % a  
DH(:,2) = zeros(3,1);           % alpha
DH(:,3) = zeros(3,1);           % d
DH(:,4) = q;                    % theta
q_des = rand(3,1);              % desired joint configuration

[Q1, E1] = prepare_data(DH, jtype, q, q_des, 2, "1");

% Check the final error between the current end-effector pose and the desired one
Final_q = Q1(:, end)
Final_errors = E1(:, end)

% Simulate the robot at each step of the iterative inverse kinematics 
% and observe how the end-effector reaches the desired pose
figure("Name","EXAMPLE 1: Three-link Planar Arm");
for steps = 1:size(Q1,2)
    sim_robot(DH,Q1(:,steps),jtype);
end

%% EXAMPLE 2: 3d arm %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all;close all;clc;

q2 = rand(3,1);
jtype2 = [0;0;0];
DH2(:,1) = rand(3,1);            % a  
DH2(:,2) = rand(3,1);           % alpha
DH2(:,3) = zeros(3,1);           % d
DH2(:,4) = q2;                    % theta
q_des2 = rand(3,1);              % desired joint configuration

% close all;clc;
[Q2, E2] = prepare_data(DH2, jtype2, q2, q_des2, 3, "2");

% Check the final error between the current end-effector pose and the desired one
Final_q = Q2(:, end)
Final_errors = E2(:, end) 

% figure("Name","EXAMPLE 2: 3d Arm");
% for steps = 1:size(Q2,2)
%     sim_robot(DH2,Q2(:,steps),jtype2);
% end
