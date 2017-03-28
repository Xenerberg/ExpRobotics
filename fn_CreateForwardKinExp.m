function [forKin] = fn_CreateForwardKinExp(theta,om,q)
    forKin = eye(4,4);
    for iCount = 1:6
        t_1 = fn_CreateTwist(om(:,iCount),q(:,iCount),0);
        exp_t = fn_CreateExpTwist(t_1,theta(iCount));    
        forKin = forKin*exp_t;
    end
end