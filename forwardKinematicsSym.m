function tforms = forwardKinematicsSym(q,rho,dh)
n = numel(q);
if isa(q,'sym')
    tform_previous = sym(eye(4));
    tforms = sym(zeros(4,4,n));
else
    tform_previous = eye(4);
    tforms = zeros(4,4,n);
end
for i = 1:n
    if rho(i)
        joint_trans = rot(q(i),'z');
    else
        joint_trans = transl(q(i),'z');
    end
%     joint_trans = (rho(i)*rot(q(i),'z')+(1-rho(i))*transl(q(i),'z')); %
%     alternative
    DH_trans = joint_trans*rot(dh(i,1),'z')*transl(dh(i,2),'z')*transl(dh(i,3),'x')*rot(dh(i,4),'x');
    tform_previous = tform_previous*DH_trans;
    if isa(q,'sym')
        tform_previous = simplify(tform_previous);
    end
    tforms(:,:,i) = tform_previous;
end
end 