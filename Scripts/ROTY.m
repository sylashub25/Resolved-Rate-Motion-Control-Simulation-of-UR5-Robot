function [ rot_mat ] = ROTY( theta )
    rot_mat = [cos(theta) 0 sin(theta); 
                        0 1 0;
              -sin(theta) 0 cos(theta)];
end

