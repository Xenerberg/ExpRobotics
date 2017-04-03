function [z] = fn_TwistToVect(Z)    
    omega = [-Z(2,3);Z(1,3);-Z(1,2)];
    v = [Z(1:3,4)];
    z = [v;omega];
end