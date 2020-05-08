function traj = evalCubic(t,coef,start_time,total_time,mode)
m = numel(start_time)+1;
if strcmpi(mode,'cyclic')||strcmpi(mode,'round+cyclic') 
    t_mod = mod(t,total_time);
    traj_idx = discretize(t_mod,[start_time;total_time]);
    tau = t_mod-start_time(traj_idx)';
    notCyclic = false;
else
    traj_idx = discretize(t,[start_time;total_time]);
    out_of_bound = ~(traj_idx<m);
    num_out = sum(out_of_bound);
    num_in = numel(t)-num_out;
    
    tau_in = t(1:num_in)-start_time(traj_idx(~out_of_bound))';
    tau_out = (total_time-start_time(m-1))*ones(num_out,1);
    tau = [tau_in tau_out'];
    traj_idx(out_of_bound) = (m-1)*ones(num_out,1);
    notCyclic = true;
end
C = coef(:,:,traj_idx);
[m,n,k] = size(C);
B = kron(sparse(eye(k)),ones(m,n));
B(B > 0) = C;
n = size(coef,1);
q = reshape(B*reshape([ones(size(tau));tau;tau.^2;tau.^3],[],1),n,[]);
v = reshape(B*reshape([zeros(size(tau));ones(size(tau));2*tau;3*tau.^2],[],1),n,[]);
a = reshape(B*reshape([zeros(size(tau));zeros(size(tau));2*ones(size(tau));6*tau],[],1),n,[]);
if and(any(t >= total_time),notCyclic)
    a = a.*(t<total_time);
end
traj = cat(3,q,v,a);
end


