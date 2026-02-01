% rot2RPY: Returns the roll, pitch and yaw corresponding to a given
% rotation matrix.
%
% [roll, pitch, yaw] = rot2RPY(R_zyx) 
% Returns the roll, pitch and yaw corresponding to a given rotation matrix. 
% It should return the two valid solutions corresponding to the +sqrt and 
% -sqrt. Each output is then a 2x1 vector with the plus solution on top. 
%
% yaw = rotation angle about z-axis
% pitch = rotation angle about y-axis
% roll = rotation angle about x-axis
%
% R = Combined rotation matrix representing a ZYX rotation
%
% Noah Chapman
% 10883958
% MEGN544
% 10/11/2025
function [roll, pitch, yaw] = rot2RPY(R)

    pitch_pos = atan2(-R(3,1), sqrt(R(1,1)^2 + R(2,1)^2));
    pitch_neg = atan2(-R(3,1), -sqrt(R(1,1)^2 + R(2,1)^2));

    roll_pos = atan2(R(3,2), R(3,3));
    roll_neg = atan2(-R(3,2), -R(3,3));

    yaw_pos = atan2(R(2,1), R(1,1));
    yaw_neg = atan2(-R(2,1), -R(1,1));

    % Check for degeneracy
    if (isapprox(mod(pitch_pos, pi/2), 0))
        if R(3,1) < 0
            % pitch = +pi/2
            roll_pos = atan2(R(1,2), R(2,2));
            roll_neg = roll_pos;
        else
            % pitch = -pi/2
            roll_neg = atan2(-R(1,2), R(2,2)); 
            roll_pos = roll_neg;
        end
    end

    roll  = [roll_pos; roll_neg];
    pitch = [pitch_pos; pitch_neg];
    yaw   = [yaw_pos; yaw_neg];
end
