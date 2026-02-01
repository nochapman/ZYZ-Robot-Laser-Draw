% pq2DualQuat(pos, quat): Returns a dual quaternion given position and quat
%
% dual_quat = pq2DualQuat(pos, quat)
% Returns a dual quaternion stored as a structure with members rot, a 4x1
% unit quaternion staked [q0;q_vec],and disp, a 4x1 quaternion stacked 
% [q0;q_vec]that encodes the dual rotation-mixed translation.
%
% dual_quat = the dual quaternion stacked [0;q_vec]
%
% pos = 3x1 position vector
% quat = 4x1 unit rotation quaternion
%
% Noah Chapman
% 10883958
% MEGN544
% 10/11/2025
function dual_quat = pq2DualQuat(pos, quat)
    dual_quat.rot = quat;
    dual_quat.disp = 0.5 * multiplyQuat([0; pos(:)], quat); 
end