function J = manipulatorJacobianRST(q,rho,dh)
%MANIPULATORJACOBIANRST computes manipulator Jacobian matrices of a 
%manipulator
%   J = MANIPULATORJACOBIANRST(q,rho,DH_table), given a joint configuration 
%   "q", a list of joint types "rho", and a table of DH parameters 
%   "DH_tble", computes a 6 x n x n manipulator Jacobian matrices "J" of 
%   the manipulator for every coordinate frames except for the base.

n = numel(q);
[~,robot,config] = forwardKinematicsRST(q,rho,dh);

%% modify here
J = zeros(6, n, n);
%iterate for each jacobian
for i = 1:n
    J(:,:, i) = geometricJacobian(robot, config, "link_" + (i));
end

end