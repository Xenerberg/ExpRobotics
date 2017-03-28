function [g_q] = fn_ComputeJSg(theta,om,q)
    ScriptForKin;
    g_a = [0;0;-9.81;0;0;0];
    g_q = zeros(6,1);    
    exp_t = eye(4,4);
    for iCount = 1:6
        g = g_com(:,:,iCount);
        t = fn_CreateTwist(om(:,iCount),q(:,iCount),0);
        exp_t = exp_t*fn_CreateExpTwist(t,theta(iCount));
        J_i = fn_Jacobian(theta,om,q,2,g,iCount);
        temp_g = exp_t*g;
        R = temp_g(1:3,1:3);
        g_a_local = [R'*g_a(1:3);zeros(3,1)];
        g_q = g_q + J_i'*M_mat(:,:,iCount)*g_a_local;
    end
    g_q = -g_q;
end