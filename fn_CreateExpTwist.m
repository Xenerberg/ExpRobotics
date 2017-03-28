function [expTwist] = fn_CreateExpTwist(twist,theta)
    omega = twist(4:6);
    v = twist(1:3);
    m11 = fn_CreateExpOmega(omega,theta);
    m12 = (eye(3,3) - m11)*cross(omega,v);
    expTwist = [m11,m12;zeros(1,3),1];
end