% quat2Rot: Returns the rotation matrix that corresponds to the quaternion,
% stacked  [q0;q_vec]
%
% R = quat2Rot(Q)
%
% R = The rotation matrix that corresponds to the given Q
%
% Q = The given quaternion
%
% Noah Chapman
% 10883958
% MEGN544
% 10/11/2025
function R = quat2Rot(Q)
    R = [1 - 2*(Q(3)^2 + Q(4)^2), 2*(Q(2)*Q(3) - Q(1)*Q(4)), 2*(Q(2)*Q(4) + Q(1)*Q(3));
        2*(Q(2)*Q(3) + Q(1)*Q(4)), 1 - 2*(Q(2)^2 + Q(4)^2), 2*(Q(3)*Q(4) - Q(1)*Q(2));
        2*(Q(2)*Q(4) - Q(1)*Q(3)), 2*(Q(3)*Q(4) + Q(1)*Q(2)), 1 - 2*(Q(2)^2 + Q(3)^2)];
end