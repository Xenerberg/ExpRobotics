function [C] = fn_ComputeCMat(theta,q,om,theta_dot)
    C = zeros(6,6);
    for i = 1:6
       for j = 1:6
          temp = 0;
          for k = 1:6
             del_m_ijk = fn_Del_m(theta,q,om,i,j,k);
             del_m_ikj = fn_Del_m(theta,q,om,i,k,j);
             del_m_kji = fn_Del_m(theta,q,om,k,j,i);             
             temp = temp + (del_m_ijk + del_m_ikj - del_m_kji)*theta_dot(k);
          end
          C(i,j) = 0.5*temp;
       end        
    end
end