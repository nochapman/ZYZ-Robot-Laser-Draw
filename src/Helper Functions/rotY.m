% rotY: Returns a rotation matrix describing a rotation about the Y axis 
% (theta in radians)
%
% [R_y] = rotY(theta) 
% A multiline description of the function would be appropriate.
%
% R_y = The rotation matrix corresponding to a rotation theta around Y-axis
%
% theta = The angle in radians to rotate by
%
% Noah Chapman
% 10883958
% MEGN544
% 10/11/2025
function R_y = rotY(theta)
    R_y = [cos(theta), 0, sin(theta);
           0,          1,          0;
          -sin(theta), 0, cos(theta)];
end
