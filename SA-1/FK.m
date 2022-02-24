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
Tn = [T];
%% T homogeneous matrix calculations
for i = 1:n 
    if jtype(i) == 1    % prismatic
        %%%% complete here %%%%
        theta = DH_params(i, 4);  
        d = q(i) + DH_params(i, 3); 
    else                % revolute
        %%%% complete here %%%%
        theta = q(i) + DH_params(i, 4);
        d = DH_params(i, 3);
    end
    %%%% complete here %%%% 
    a = DH_params(i, 1);
    alph = DH_params(i, 2); % alpha
    c_alpha = cos(alph);
    s_alpha = sin(alph);
    c_theta = cos(theta);
    s_theta = sin(theta); 
    % computing Homogeneous matrix
    A_i = [                 % Equation. 1
        c_theta, -s_theta*c_alpha, s_theta*s_alpha, a*c_theta;
        s_theta, c_theta*c_alpha, -c_theta*s_alpha, a*s_theta;
        0, s_alpha, c_alpha, d;
        0, 0, 0, 1];
    %%%% complete here %%%%
    T = T * A_i;            % Equation. 2
    Tn = cat(3, Tn , T);    % save intermediate for J
end

%% J Jacobian calculations
for i = 1:n
    Z_i = Tn(1:3, 3, i);
    if jtype(i) == 1    % prismatic
        %%%% complete here %%%%
        J(:,i) = [      % Equation. 3
            Z_i;        % linear 
            0; 0; 0];   % angular 
    else                % revolute
        %%%% complete here %%%%
        J(:,i) = [      % Equation. 3
            cross(Z_i, T(1:3, 4) - Tn(1:3, 4, i)); % linear 
            Z_i];       % angular 
    end
end
