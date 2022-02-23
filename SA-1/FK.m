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
    a = DH_params(i, 1);
    alph = DH_params(i, 2);
    c_alpha = cos(alph);
    s_alpha = sin(alph);

    if jtype(i) == 1  % prismatic
        %%%% complete here %%%%
        d = DH_params(i, 3);
        A_i = [
            1, 0, 0, a;
            0, c_alpha, -s_alpha, 0;
            0, s_alpha, c_alpha, d;
            0, 0, 0, 1];
    else            % revolute
        %%%% complete here %%%%
        theta = DH_params(i, 4);
        c_theta = cos(theta);
        s_theta = sin(theta);
        A_i = [
            c_theta, -s_theta*c_alpha, s_theta*s_alpha, a*c_theta;
            s_theta, c_theta*c_alpha, -c_theta*s_alpha, a*s_theta;
            0, s_alpha, c_alpha, 0;
            0, 0, 0, 1];
    end
    %%%% complete here %%%%
    % computing Homogeneous matrix
    T = T * A_i;
    %%%% complete here %%%%
    Tn = cat(3, Tn , T); % save intermediate for J
end

%% J Jacobian calculations
for i = 1:n
    Z_i = Tn(1:3,3,i);
    if jtype(i) == 1  % prismatic
        %%%% complete here %%%%
        J(:,i) = [
            Z_i; % linear velocity
            0; 0; 0]; % angular velocity
    else             % revolute
        %%%% complete here %%%%
        J(:,i) = [
            cross(Z_i, T(1:3,4) - Tn(1:3,4,i)); % linear velocity
            Z_i]; % angular velocity
    end
end
