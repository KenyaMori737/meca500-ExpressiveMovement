function [R] = rotz(ang)
    ang_rad = ang*pi/180;
    R = eye(3);
    R(1,1) = cos(ang_rad); R(1,2) = -sin(ang_rad); 
    R(2,1) = sin(ang_rad); R(2,2) =  cos(ang_rad);
    
end