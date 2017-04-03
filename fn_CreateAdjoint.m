function [Adj] = fn_CreateAdjoint(H,flag)
    R = H(1:3,1:3);
    p = H(1:3,4);
    switch(flag)
        case 0% 0: normal adjoint                        
            Adj = [R, fn_VectorToSkewSymmetricTensor(p)*R;zeros(3,3),R];
        case 1 %1: inv adjoint
            Adj = [R', -R'*fn_VectorToSkewSymmetricTensor(p);zeros(3,3),R'];
        case 2 %2: Pose twist adjoint
            Adj = [R, zeros(3,3);zeros(3,3), R];
    end
end