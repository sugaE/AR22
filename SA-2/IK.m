function [Q, E] = IK(DH_params, jtype, q, pdes, odes)
% IK calculates the inverse kinematics of a manipulator
% Q = IK(DH_params, jtype, q, pdes, odes) calculates the joint
% values of a manipulator robot given the desired end-effector's position
% and orientation by solving the inverse kinematics, iteratively.  The
% inputs are DH_params, jtype and q, pdes, and odes.  DH_params is
% nx4 matrix including Denavit-Hartenberg parameters of the robot.  jtype
% and q are n-dimensional vectors.  jtype describes the joint types of the
% robot and its values are either 0 for revolute or 1 for prismatic joints.
% q is the joint values.  pdes and odes are desired position and
% orientation of the end-effector.  pdes is 2x1 for a planar robot and 3x1
% for a spatial one.  odes is a scalar for a planar robot and a 3x1 vector
% for a 3D robot.  Orientation is defined as roll-pitch-yaw angles.  Use
% Jacobian Transpose to compute the inverse of the Jacobian.

n = size(q,1);  % robot's DoF
% consistency check
if (n~=size(DH_params,1)) || (n~=size(jtype,1))
    error('inconsistent in dimensions');
end

% robot's dimension (planar or spatial)
dim = size(pdes,1);
if ~((dim==2) || (dim==3))
    error('size of pdes must be either 2x1 for a planar robot or 3x1 for a spatial robot');
end

% check the size of orientation parameter
if dim==2
    if size(odes,1)~=1
        error('desired orientation must be scalar for a planar robot');
    end
else
    if size(odes,1)~=3
        error('desired orientation must be 3x1 for a spatial robot');
    end
end

%% iterative inverse kinematics
%%% Complete here %%% 
ind = 1;
% end term: max iterations
max_ind = 1500;
% end term: error threshod
err_thred = 0.05;
if dim==2
    K = eye(dim+1);
else
    K = eye(dim+3);
end
% converging rate
K = K * 0.05; %2d

% q after each iteration
Q = [q];
% error after each iteration
E = [];

while ind <= max_ind  
    q1 = Q(:,end);
    [T, J] = FK(DH_params, jtype, q1);
    p_e = T(1:dim, 4); % position
    R_e = T(1:3, 1:3); % rotation matrix
    
    phi_e = atan2(R_e(2,1), R_e(1,1));
    theta_e=0;
    psi_e=0;
    o_e = phi_e;
    if dim ~= 2
        theta_e = atan2(-R_e(3,1), sqrt(R_e(3,2)^2+R_e(3,3)^2));
        psi_e = atan2(R_e(3,2), R_e(3,3));
        o_e = [theta_e;psi_e;phi_e];
    end

    err_pos = pdes - p_e;
    err_ori = odes - o_e;

    errs = [err_pos; err_ori];
    E = cat(2, E, errs);
    if norm(err_pos) < err_thred & norm(err_ori) < err_thred
        fprintf(num2str(ind)+" iters in total.\n");
        return;
    end
 
    if dim==2
        J = cat(1,J(1:2,:),J(end,:));
        Ja = J; 
    else
        T_phi = eye(6);

        Ta = [
            0,-sin(phi_e), cos(phi_e)*cos(theta_e);
            0, cos(phi_e), sin(phi_e)*cos(theta_e);
            1,0,-sin(theta_e)];

        T_phi(end-2:end,end-2:end)=Ta;
%         Analysis Jaccobian. 
%         Ja  =  inv(T_phi) * J;
        Ja  = T_phi * J;
    end  
     
    qdot = Ja' * K * errs;
    % q(t+1) = q(t) + qdot(t) * \delta t
    q1 = q1 + qdot; 

    % wrap angles for revolute joints
    angleraw = (q1 > pi) & ~jtype;
    q1(angleraw) = q1(angleraw) - 2*pi;
    angleraw = (q1 < -pi) & ~jtype;
    q1(angleraw) = q1(angleraw) + 2*pi;

    Q = cat(2, Q, q1);
    ind = ind+1;
end

fprintf(num2str(ind)+" iters. Point maybe unreachable.\n");
