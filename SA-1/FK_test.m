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
[T1, J1] = FK(DH, jtype, q); 
% Plot robot 
figure()
sim_robot(DH,q,jtype)

%% Other examples
% set up here the code to test your FK function with other robot's
% configuration/DH parameters

