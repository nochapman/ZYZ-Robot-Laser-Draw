% dualQuat2PQ(dual_quat): Returns a position and a quaternion that 
% represents the transform encoded in the dual quaternion structure
%
% [pos, quat] = dualQuat2PQ(dual_quat)
% Returns a position and a quaternion that represents the transform encoded 
% in the dual quaternion structure, which has members rot and disp.
%
% pos = 3x1 position vector
% quat = 4x1 unit rotation quaternion
%
% dual_quat = the dual quaternion stacked [0;q_vec]
%
% Noah Chapman
% 10883958
% MEGN544
% 10/11/2025
function [pos, quat] = dualQuat2PQ(dual_quat)

    r = dual_quat.rot;
    d = dual_quat.disp;

    r_star = [r(1); -r(2:4)];
    t_quat = 2 * multiplyQuat(d, r_star);

    pos = t_quat(2:4);
    quat = r;
end