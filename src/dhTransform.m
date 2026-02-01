% twist2Transform(t): Returns the homogeneous transform corresponding to
% the provided DH parameters for a link.
%
% H = dhTransform(a, d, alpha, theta)
%
% H = the homogeneous transformation matrix
%
% a = 
% d = 
% alpha = 
% theta = 
%
% Noah Chapman
% 10883958
% MEGN544
% 10/11/2025
function H = dhTransform(a, d, alpha, theta)
H = [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta);
     sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
     0, sin(alpha), cos(alpha), d;
     0, 0, 0, 1];
end
