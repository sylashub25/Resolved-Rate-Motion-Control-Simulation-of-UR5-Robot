function [ rot_mat ] = ROTZ( theta )
    rot_mat = [cos(theta) -sin(theta) 0; 
                sin(theta) cos(theta) 0; 
                                  0 0 1];
end

