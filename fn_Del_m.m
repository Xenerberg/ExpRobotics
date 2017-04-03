function [del_m] = fn_Del_m(theta,q,om,i,j,k)
    max_ind = max([i,j]);
    temp = 0;
    A_ki = fn_AdjointFunc(theta,q,om,k,i);
    A_kj = fn_AdjointFunc(theta,q,om,k,j);
    t_i = (fn_CreateTwist(om(:,i),q(:,i),0));
    t_k = (fn_CreateTwist(om(:,k),q(:,k),0));
    t_j = (fn_CreateTwist(om(:,j),q(:,j),0));
    for l = max_ind:6       
       A_lk = fn_AdjointFunc(theta,q,om,l,k);
       A_lj = fn_AdjointFunc(theta,q,om,l,j);
       A_li = fn_AdjointFunc(theta,q,om,l,i);      
       
       lb_1 = fn_LieBracket(A_ki*t_i,t_k);
       lb_2 = fn_LieBracket(A_kj*t_j,t_k);
       M_l = fn_GetReflMass(l,theta);
       temp = temp + lb_1'*A_lk'*M_l*A_lj*t_j + t_i'*A_li'*M_l*A_lk*lb_2;       
    end
    del_m = temp;
end