function [Z] = fn_VectToTwist(z)    
    omega = z(4:6);
    v = z(1:3);
    Z = [fn_VectorToSkewSymmetricTensor(omega),v;zeros(1,4)];
end