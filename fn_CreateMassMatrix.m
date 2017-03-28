function [M] = fn_CreateMassMatrix(theta,om,q)
    ScriptForKin;
    M = zeros(6,6);
    for iCount = 1:6
        g = g_com(:,:,iCount);
        J_i = fn_Jacobian(theta,om,q,2,g,iCount);
        M_i = J_i'*M_mat(:,:,iCount)*J_i;
        M = M + M_i;
    end
end

