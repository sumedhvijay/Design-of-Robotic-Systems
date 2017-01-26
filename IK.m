function inverse = IK(EE_coordinates)

findJ = jacobian_Matrix();

% Find estimate to obtain threshold limit for while loop.
q0 =[0;0;0;0];
T0 = FK(q0(1), q0(2),q0(3),q0(4));
x = EE_coordinates - T0;
q = q0;

while norm(x) > .01    
    t1 = q(1);
    t2 = q(2);
    t3 = q(3);
    t4 = q(4);
    digits(6);
    J = vpa(subs(findJ));
    dq = pinv(J)*x;
    q = q+dq;
    T = FK(q(1), q(2),q(3),q(4));
    x = EE_coordinates - T;
end

inverse = vpa(mod(q,2*pi));

