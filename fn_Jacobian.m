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
        case 4
            ScriptForKin;
            g = eye(4,4);
            g_temp = g;
            H = fn_CreateForwardKinExp(theta,om,q)*g_st_0;
            ee_pos = H(1:3,4);
            for iCount1 = (1:6) 
                g_temp = eye(4,4);
                for i = 1:iCount1
                   R_temp = fn_GetRot(theta(i),om(:,i));
                   p_temp = local_l(:,i);
                   g_temp = g_temp*[R_temp, p_temp;zeros(1,3),1]; 
                end
                om_g = sign(om(abs(om(:,iCount1)) == 1,iCount1))*g(1:3,find(abs(om(:,iCount1))==1));
                r = g_temp(1:3,4);
                J_i = [cross(om_g,ee_pos-r);om_g];
                J(:,iCount1) = J_i;
                
                R = fn_GetRot(theta(iCount1),om(:,iCount1));
                p = local_l(:,iCount1);
                g = g*[R, p;zeros(1,3),1];
            end
            
    end
end