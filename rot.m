function H = rot(ang,ax)
%ROT creates a transformation matrix for pure rotatio nin 3d-space
%   ROT(ANG,AX) creates a transformation matrix H in 3d-space given an angle ANG
%   and an axis of rotation AX ['x','y', or 'z'].
if isa(ang,'sym')
    H = sym(eye(4));
else
    H = eye(4);
end

c = cos(ang);
s = sin(ang);

if strcmpi(ax,'x')
    H(1:3,1:3) = [1 0 0 ; 0 c -s; 0 s c];
elseif strcmpi(ax,'y')
    H(1:3,1:3) = [c 0 s ; 0 1 0 ; -s 0 c];
elseif strcmpi(ax,'z')
    H(1:3,1:3) = [c -s 0; s c 0 ; 0 0 1];
else
    error('incorrect axis of rotation')
end

end

