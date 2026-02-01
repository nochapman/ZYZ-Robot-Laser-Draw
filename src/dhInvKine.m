% dhInvKine: Returns the parameter list necessary to achieve a desired 
% homogeneous transform and the residual error.
%
% [paramList, error] = dhInvKine(linkList, desTransform, paramListGuess)
% Returns the parameter list, according to the robot's encoders, 
% necessary to achieve a desired homogeneous transform and the residual error.
%
% paramList = current θ or d values for the joints, according to the
% robot’s encoders (N × 1 array)
% error = 
%
% linkList = list of the joint parameters created by createLink
% desTransform = desired homogeneous transform
% paramListGuess = initial guess at the parameters, according to the 
% robot’s encoders
%
% Noah Chapman
% 10883958
% MEGN544
% 11/22/2025
function [paramList, error] = dhInvKine(linkList, desTransform, paramListGuess)

    paramList = paramListGuess;
    
    for iter = 1:100
        Tc = dhFwdKine(linkList, paramList);

        % Error calculations
        e_pos = desTransform(1:3,4) - Tc(1:3,4);
        e_rot = rot2AngleAxis(desTransform(1:3,1:3)*Tc(1:3,1:3)');

        error = [e_pos;e_rot];
        e_norm = norm(error);

        % Compute J_inv
        [U, E, V] = svd(velocityJacobian(linkList, paramList));
        J_inv = V*pinv(E)*transpose(U);

        delta_q = J_inv*error;
        paramList = paramList + delta_q;

        if e_norm < 1e-12 || norm(delta_q) < 1e-12
            return;
        end
    end
end