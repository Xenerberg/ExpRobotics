function [velScrew,velPoint] = fn_VelocityScrew(J,dtheta,p)
    velScrew = J*dtheta;
    if nargin < 3
        p = [0;0;0];       
    end        
    twistMat = [fn_VectorToSkewSymmetricTensor(velScrew(4:6)),velScrew(1:3);zeros(1,3),1];
    velPoint = twistMat*[p;1];
    velPoint = velPoint(1:3);
end