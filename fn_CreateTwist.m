function [twist] = fn_CreateTwist(axis, point,flag)
    switch (flag)
        case 0
            twist(1:3,1) = -cross(axis,point); %omega
            twist(4:6,1) = axis;
        case 1
            twist(1:3,1) = axis; %velocity axis for prismatic joints
            twist(4:6,1) = zeros(3,1);
    end
end