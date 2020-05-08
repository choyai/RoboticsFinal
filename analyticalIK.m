function q = analyticalIK(p)
    a_1 = 0.2;
    a_2 = 0.75;
    a_3 = 0.6;
    d_1 = 0.9;
    rho = [1 1 1];
    dh = [0 d_1 a_1 pi/2;pi/2 0 a_2 0;-pi/2 0 a_3 -pi/2];
    q = [0;0;0];
    
    x = p(1);
    y = p(2);
    z = p(3);
    A = sqrt(x^2 + y^2) - a_1;
    B = z - d_1;
    q(1) = atan2(y/(a_1 + A),x/(a_1 + A));
    D = (A^2 + B^2 - a_2^2 - a_3^2)/(2*a_2*a_3);
    q(3) = atan2(sqrt(1-D^2),D)+pi/2;
    beta = atan2(a_3*sin(q(3)),a_3*cos(q(3)) + a_2);
    q(2) = atan2(B, A) - beta - pi/2;
    if D == 1
       q = [q];
    
    else
    Q = q;
    q(3) = atan2(-1*sqrt(1-D^2),D)+pi/2;
    beta = atan2(a_3*sin(q(3)),a_3*cos(q(3)) + a_2);
    q(2) = atan2(B, A) - beta -pi/2;
    q = [Q q];
   % end
end