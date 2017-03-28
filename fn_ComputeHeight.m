function [h] = fn_ComputeHeight(theta,om,q)
    ScriptForKin;
    exp_t = eye(4,4);    
    for iCount = 1:6
        t = fn_CreateTwist(om(:,iCount),q(:,iCount),0);
        exp_t = exp_t*fn_CreateExpTwist(t,theta(iCount));
        temp =  exp_t*g_com(:,:,iCount);
        h(:,iCount) = temp(1:3,4);
    end
end