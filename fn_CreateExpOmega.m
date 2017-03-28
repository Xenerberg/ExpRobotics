function [expOmega] = fn_CreateExpOmega(omega,theta)   
    expOmega = eye(3,3) + fn_VectorToSkewSymmetricTensor(omega)*sin(theta) + fn_VectorToSkewSymmetricTensor(omega)^2*(1-cos(theta));
end
