function [tforms,robot,config] = forwardKinematicsRST(q,rho,dh)
n = numel(q);
robot = rigidBodyTree;
link_previous = 'base';
internal_limits = [-pi pi ; -pi pi; -pi pi];

for i = 1:n
    link_name = sprintf('link_%d',i);
    joint_name = sprintf('joint_%d',i);
    body = rigidBody(link_name);
    if rho(i)
        joint_type = 'revolute';
    else
        joint_type = 'prismatic';
    end
    jnt = rigidBodyJoint(joint_name,joint_type);
    jnt.PositionLimits = internal_limits(i,:);
    jnt.HomePosition = rho(i)*dh(i,1)+(1-rho(i))*dh(i,2);
    jnt.setFixedTransform(dh(i,[3 4 2 1]),'dh');
    body.Joint = jnt;
    
    addBody(robot,body,link_previous);
    link_previous = link_name;
    
end
config = robot.homeConfiguration;
for i = 1:n
    config(i).JointPosition = q(i)+rho(i)*dh(i,1)+(1-rho(i))*dh(i,2);
end

tforms = zeros(4,4,n);
for i = 1:n
    tforms(:,:,i) = getTransform(robot,config,robot.BodyNames{i},'base');
end
% display frame at that config
%show(robot,config);
end