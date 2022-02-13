function [T, J] = FK(DH_params, jtype, q)
% FK calculates the forward kinematics of a manipulator
% [T, J] = FK(DH_params, jtype, q) calculates the Homogeneous
% transformation matrix from base to the end-effector (T) and also the
% end-effector Jacobian with respect to the base frame of a manipulator
% robot.  The inputs are DH_params, jtype and q.  DH_params is nx4 matrix
% including Denavit-Hartenberg parameters of the robot.  jtype and q are
% n-dimensional vectors.  jtype describes the joint types of the robot.
% Its values are either 0 for revolute or 1 for prismatic joints.  q is the
% vector of joint values.

n = size(q,1);  % robot's DoF
% consistency check
if (n~=size(DH_params,1)) || (n~=size(jtype,1))
    error('inconsistent in dimensions');
end

% initialisation
T = eye(4,4);
J = zeros(6,n);
%% T homogeneous matrix calculations
for i = 1:n
    if jtype(i)==1  % prismatic
        %%%% complete here %%%%
    else            % revolute
        %%%% complete here %%%%
    end
    %%%% complete here %%%%
    % computing Homogeneous matrix
    %%%% complete here %%%%
end

%% J Jacobian calculations
for i = 1:n  
    if jtype(i)==1  %prismatic
        %%%% complete here %%%%
    else             % revolute
        %%%% complete here %%%%
    end  
end
