function H = transl(dist,ax)
%TRANSL creates a transformation matrix for pure translation in 3d-space
%   TRANSL(DIST,AX) creates a transformation matrix H in 3d-space given a
%   displacement DIST and an axis of translatioh AX ['x','y', or 'z'].
if isa(dist,'sym')
    H = sym(eye(4));
else
    H = eye(4);
end

if strcmpi(ax,'x')
    idx = 1;
elseif strcmpi(ax,'y')
    idx = 2;
elseif strcmpi(ax,'z')
    idx = 3;
else
    error('incorrect axis of rotation')
end
H(idx,4) = dist;

end

