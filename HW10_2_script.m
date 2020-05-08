
q = [0 1 3 -1 0];
dt = [0.5 1 0.75 1];
qdot = [0 0 0 0 0];
c0 = [0 0 0 0];
c1 = [0 0 0 0];
c2 = [0 0 0 0];
c3 = [0 0 0 0];

for i = 1:size(qdot, 2) - 2
    qdot(i+1) = (q(i+1) - q(i))/dt(i);
end
for j = 1:4
    c0(j) = q(j);
    c1(j) = qdot(j+1);
    c2(j) = 3*(q(j+1) -q(j))/(dt(j)^2) - (qdot(j+1) - 2*qdot(j))/dt(j);
    c3(j) = (2*q(j) - 2*q(j+1))/(dt(j)^3) + (qdot(j+1) + qdot(j))/(dt(j)^2);
end
C = cat(3, c0,c1,c2,c3);
start_time = [0;cumsum(dt(1:end-1))'];
total_time = sum(dt);
mode = 'regular'; % regular, round, cyclic

traj = @(t)evalCubic(t,C,start_time,total_time,mode);
m = 1;
g = 9.80665;
l = 0.25;

KP = 50;
KD = 10;
a_q = @(q,v,traj)traj(:,:,3)+KD*(traj(:,:,2)-v)+KP*(traj(:,:,1)-q);
u = @(t,q,v,traj)(m*l^2)*a_q(q,v,traj(t))+m*g*l*sin(q);
t_max = 5;
q0 = 0;
v0 = 0;

[t,x] = ode45(@(t,x)[x(2);-g/l*sin(x(1))+1/(m*l^2)*u(t,x(1),x(2),traj)],[0 t_max],[q0;v0]);

ref = traj(t');
hold on;
plot(t,ref(:,:,1),'k--');
plot(t,x(:,1),'b');