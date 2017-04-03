function [R] = fn_GetRot(theta,om)
    axis = find(abs(om)==1);
    theta = sign(om(abs(om) == 1))*theta;
    switch(axis)
        case 1
            R = [1,0, 0;0 cos(theta) -sin(theta);0 sin(theta) cos(theta)];
        case 2
            R = [cos(theta) 0 sin(theta);0, 1, 0; -sin(theta) 0 cos(theta)];
        case 3
            R = [cos(theta) -sin(theta) 0;sin(theta) cos(theta) 0;0, 0, 1];
    end
end