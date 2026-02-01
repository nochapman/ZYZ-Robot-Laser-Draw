% angleAxis2Rot: Returns the rotation matrix encoded by an angle-axis 
% (Omega = theta*k) rotation of theta radians about the unit vector k axis. 
%
% R = angleAxis2Rot(Omega)
%
% R = The rotation matrix
%
% Omega = The angle-axis 
%
% Noah Chapman
% 10883958
% MEGN544
% 10/11/2025
function R = angleAxis2Rot(Omega)
    if Omega == 0
        R = eye(3); % Identity matrix for zero rotation
        return
    end

    % Extract the rotation angle
    theta = norm(Omega);
   
    %Calculate the unit vector
    k = Omega / theta;
    R = eye(3) + sin(theta) * cpMap(k) + (1 - cos(theta)) * (cpMap(k) ^ 2);
end