function q = inverseKinematics3DOF(p, q_guess)
a_1 = 0.2;
a_2 = 0.75;
a_3 = 0.6;
d_1 = 0.9;

dh=[0 d_1 a_1 pi/2;pi/2 0 a_2 0;-pi/2 0 a_3 -pi/2];
rho=[1 1 1];
q = [0;0;0];
[~,robot,~] = forwardKinematicsRST(q,rho,dh);

% inverse kinematics
ik = inverseKinematics('RigidBodyTree',robot);
H = [eye(3) p; 0 0 0 1]; % transformation matrix; 
%H = trvec2tform(p); % built-in function
weights = [0 0 0 1 1 1];
initialguess = homeConfiguration(robot);
for i = 1:numel(q_guess)
    initialguess(i).JointPosition = q_guess(i);
end
[configSoln,~] = ik('link_3',H,weights,initialguess);
qSoln = zeros(numel(q),1);
for i = 1:numel(q)
    qSoln(i) = configSoln(i).JointPosition-rho(i)*dh(i,1)-(1-rho(i))*dh(i,2);  % take offset into account
end
q = qSoln
% verify the result by forward kinematics
[tforms,~,~] = forwardKinematicsRST(qSoln,rho,dh);
err = round(abs(tforms(1:3,4,2)-p),4)

end