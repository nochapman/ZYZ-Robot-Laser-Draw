% rot2Quat: Returns the quaternion [qo;q_vec] that corresponds to 
% the rotation matrix. 
%
% Q = rot2Quat(R)
%
% Q = The quaternion that corresponds to R
%
% R = The given rotation matrix
%
% Noah Chapman
% 10883958
% MEGN544
% 10/11/2025
function Q = rot2Quat(R)
    % Same calculations as from rot2AngleAxis, faster to reuse the code
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
    Q(1) = cos(theta/2);
    Q(2) = k(1)*sin(theta/2);
    Q(3) = k(2)*sin(theta/2);
    Q(4) = k(3)*sin(theta/2);
    Q = Q(:);
end