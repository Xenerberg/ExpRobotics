function [H] = fn_FetchTransform(i,j,theta,flag)%j w.r.t i
    max_ind = max([i,j]);
    min_ind = min([i,j]);
    ScriptForKin;
    H = eye(4,4);
    if max_ind == min_ind
        return;
    end 
    switch(flag)
        case 0
            temp = -1;
        case 1
            temp = 0;
    end
        
       
   
    for iCount = min_ind:max_ind+temp
        R = fn_GetRot(theta(iCount),om(:,iCount));
        p = local_l(:,iCount);
        H = H*[R,p;zeros(1,3),1];
        if iCount == 6            
            H(1:3,1:3) = H(1:3,1:3)*[1,0,0;0,0,-1;0,1,0];;
        end
    end
    
    
    if i > j
        H = inv(H);
    end
end
