function [A] = fn_AdjointFunc(theta, q, om, i, j)
    if i > j
       exp_t = eye(4,4);
       for iCount = j+1:i
          t = fn_CreateTwist(om(:,iCount),q(:,iCount),0);
          exp_t = exp_t*fn_CreateExpTwist(t,theta(iCount));          
       end
       A = fn_CreateAdjoint(exp_t,1);
    elseif i == j
       A = eye(6,6);        
    elseif i < j
       A = zeros(6,6);        
    end
end