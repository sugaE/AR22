clear all;close all;clc;
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
%% EXAMPLE: planar robot
q = [pi/2;pi/4;pi/2];

DH(:,1) = [0.5 1 0.5]';             % a  
DH(:,2) = [pi/2 0 0]';           % alpha
DH(:,3) = [0 0 0]';               % d
DH(:,4) = q;      % theta

jtype = zeros(3,1);

% Forward Kinematics
[T1, J1] = FK(DH, jtype, q)
% Plot robot 
figure()
sim_robot(DH,q,jtype)

%% Other examples. Using typical manipulator structures from Robotics book 3.2
% set up here the code to test your FK function with other robot's
% configuration/DH parameters

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% BOOK 3.2.1 three-link planar arm
q2 = [pi/3;pi/2;-pi/4];

DH2(:,1) = [1 2 0.5]';             % a  
DH2(:,2) = [0 0 0]';           % alpha
DH2(:,3) = [0 0 0]';               % d
DH2(:,4) = q2;      % theta

jtype2 = zeros(3,1);

[T2, J2] = FK(DH2, jtype2, q2)
% Plot robot 
figure()
sim_robot(DH2,q2,jtype2)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% BOOK 3.2.3 Stanford Manipulator
q3 = [pi/2;pi/2;0;-pi/4;pi/3;-pi/6];

DH3(:,1) = [1.5,2,1,0.5,0.5,0.5]';             % a  
DH3(:,2) = [pi/2, -pi/2, 0, pi/2, -pi/2, pi/2]';           % alpha
DH3(:,3) = [0 0 2 0 0 0]';               % d
DH3(:,4) = q3;      % theta

jtype3 = [0 0 1 0 0 0]';

[T3, J3] = FK(DH3, jtype3, q3)
% Plot robot 
figure()
sim_robot(DH3,q3,jtype3)
