function J = manipulatorJacobianSym(q,rho,dh)
%MANIPULATORJACOBIANSYM computes manipulator Jacobian matrices of a 
%manipulator
%   J = MANIPULATORJACOBIANSYM(q,rho,DH_table), given a joint configuration 
%   "q", a list of joint types "rho", and a table of DH parameters 
%   "DH_tble", computes a 6 x n x n manipulator Jacobian matrices "J" of 
%   the manipulator for every coordinate frames except for the base.

n = numel(q);

J = zeros(6,n,n);
if isa(q,'sym')
    J = sym(J);
end

tform = forwardKinematicsSym(q,rho,dh);

%% Modify here


%iterate for each jacobian
for j = 1:n
%iterate for each joint
o_j = tform(1:3,4,j);
%base frame z and o
z_iml = [0 0 1]';
o_iml = [0 0 0]';

for i = 1:j
   %compute J_w_i and J_v_i according to formula
   J_w_i = rho(i)*z_iml;
   J_v_i = cross(J_w_i, o_j - o_iml) + (1 - rho(i))*z_iml;
   J(:,i,j) = [J_w_i;J_v_i];
   z_iml = tform(1:3,3,i);
   o_iml = tform(1:3,4,i);
end


%%
if isa(q,'sym')
    J = simplify(J);
end

end