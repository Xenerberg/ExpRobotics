function [z] = fn_LieBracket(t1,t2)
    Z_1 = fn_VectToTwist(t1);
    Z_2 = fn_VectToTwist(t2);
    Z = Z_1*Z_2 - Z_2*Z_1;
    z = fn_TwistToVect(Z);
end