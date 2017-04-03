function [M] = fn_CreateMassMatrix2(theta,om,q)
    ScriptForKin;    
    for i = 1:6
       for j = 1:6
           max_ind =  max([i,j]);
           m = 0;
           for l = max_ind:6
              t_i = fn_CreateTwist(om(:,i),q(:,i),0);
              A_li = fn_AdjointFunc(theta,q,om,l,i);
              t_j = fn_CreateTwist(om(:,j),q(:,j),0);
              A_lj = fn_AdjointFunc(theta,q,om,l,j);
              M_l = fn_GetReflMass(l,theta);
              m = m +  t_i'*A_li'*M_l*A_lj*t_j;
           end
           M(i,j) = m;
       end       
    end
end