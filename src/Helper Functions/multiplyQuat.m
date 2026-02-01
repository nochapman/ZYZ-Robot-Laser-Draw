% multiplyQuat(Q_left, Q_right): Returns the quaternion product between two
% quaternions that are stored in 4x1 vectors as [q0;q_vec]
%
% Q = multiplyQuat(Q_left, Q_right)
%
% Q = the quaternion product between Q_left and Q_right
%
% Q_left
% Q_right
%
% Noah Chapman
% 10883958
% MEGN544
% 10/11/2025
function Q = multiplyQuat(Q_left, Q_right)
    q0_left = Q_left(1); 
    qv_left = Q_left(2:4);

    q0_right = Q_right(1); 
    qv_right = Q_right(2:4);
    
    % Compute Scalar
    q0 = q0_left*q0_right - dot(qv_left, qv_right);
    
    % Compute Vector
    qv = q0_left*qv_right + q0_right*qv_left + cross(qv_left, qv_right);
    
    Q = [q0; qv];
end