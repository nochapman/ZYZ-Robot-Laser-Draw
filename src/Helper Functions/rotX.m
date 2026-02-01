% rotX: Returns a rotation matrix describing a rotation about the X axis 
% (theta in radians)
%
% [R_x] = rotX(theta) 
% A multiline description of the function would be appropriate.
%
% R_x = The rotation matrix corresponding to a rotation theta around x-axis
%
% theta = The angle in radians to rotate by
%
% Noah Chapman
% 10883958
% MEGN544
% 10/11/2025
function R_x = rotX(theta)
    R_x = [1,          0,           0; 
           0, cos(theta), -sin(theta); 
           0, sin(theta), cos(theta)];
end
