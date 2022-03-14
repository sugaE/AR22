function Q = IK(DH_params, jtype, q, pdes, odes)
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
