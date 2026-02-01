% rotZ: Returns a rotation matrix describing a rotation about the Z axis 
% (theta in radians)
%
% [R_z] = rotZ(theta) 
% A multiline description of the function would be appropriate.
%
% R_z = The rotation matrix corresponding to a rotation theta around Z-axis
%
% theta = The angle in radians to rotate by
%
% Noah Chapman
% 10883958
% MEGN544
% 10/11/2025
function R_z = rotZ(theta)
    R_z = [cos(theta), -sin(theta), 0; 
           sin(theta),  cos(theta), 0; 
           0,           0,          1];
end