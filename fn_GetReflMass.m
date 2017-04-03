function [M_refl] = fn_GetReflMass(i,theta)    
    ScriptForKin;
    g = g_com(:,:,i);
    A = fn_CreateAdjoint(g,1);
    M = M_mat(:,:,i);
    M_refl = A'*M*A;
end