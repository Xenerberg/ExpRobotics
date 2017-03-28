function [J] = fn_Jacobian(theta,om,q,flag,g_st_0,lnk_no)
    exp_t = eye(4,4);
    Adj_t = eye(6,6);
    J = zeros(6,6);
    switch(flag)
        case 0 %0: Spatial Jacobian            
            for iCount1 = 1:6              
               t = fn_CreateTwist(om(:,iCount1),q(:,iCount1),0);
               exp_t = exp_t*fn_CreateExpTwist(t,theta(iCount1));    
               J_i = Adj_t*t;
               J(:,iCount1) = J_i;
               Adj_t = fn_CreateAdjoint(exp_t,0);          
            end
        case 1 %1: Body Jacobian
            for iCount1 = fliplr(1:6)
               t = fn_CreateTwist(om(:,iCount1),q(:,iCount1),0);
               exp_t = fn_CreateExpTwist(t,theta(iCount1))*exp_t;
               Adj_t = fn_CreateAdjoint(exp_t*g_st_0,1);
               J_i = Adj_t*t;
               J(:,iCount1) = J_i;
            end
        case 2 %2: Mass center Jacobians
            for iCount1 = fliplr(1:lnk_no)
                t = fn_CreateTwist(om(:,iCount1),q(:,iCount1),0);
                exp_t = fn_CreateExpTwist(t,theta(iCount1))*exp_t;
                Adj_t = fn_CreateAdjoint(exp_t*g_st_0,1);
                J_i = Adj_t*t;
                J(:,iCount1) = J_i;
            end
        case 3 %3: Spatial Jacobian for links
            for iCount1 = 1:lnk_no              
               t = fn_CreateTwist(om(:,iCount1),q(:,iCount1),0);
               exp_t = exp_t*fn_CreateExpTwist(t,theta(iCount1));    
               J_i = Adj_t*t;
               J(:,iCount1) = J_i;
               Adj_t = fn_CreateAdjoint(exp_t,0);          
            end   
            
    end
end