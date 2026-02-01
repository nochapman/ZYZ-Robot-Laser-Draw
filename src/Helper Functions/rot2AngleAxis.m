% rot2AangleAxis: Returns the angle times axis (theta*k) corresponding to a 
% rotation matrix. 
%
% Omega = rot2AngleAxis(R)
% Returns the angle times axis (theta*k) corresponding to a rotation matrix. 
% Take care to handle rotations where sin(theta) is approximately zero properly. 
% This includes both small angles and pi multiples.
%
% Omega = The angle-axis (theta*k)
%
% R = The given rotation matrix
%
% Noah Chapman
% 10883958
% MEGN544
% 10/11/2025
function Omega = rot2AngleAxis(R)
    axis = [R(3,2)-R(2,3); R(1,3)-R(3,1); R(2,1)-R(1,2)];
    theta = atan2(.5 * norm(axis), (trace(R) - 1) / 2);
    % Special Case where theta=pi
    if mod(theta, pi) == 0
        % Somehow this simplification just works
        k = [sqrt(max(0, (R(1,1)+1)/2)); sqrt(max(0, (R(2,2)+1)/2)); sqrt(max(0, (R(3,3)+1)/2))];
    else
        % General case
        k = (1 / (2 * sin(theta))) * axis;
    end
    Omega = theta * k;
end