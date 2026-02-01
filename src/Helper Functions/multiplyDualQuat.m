% multiplyDualQuat(dq_left, dq_right): Returns the dual quaternion product 
% between two dual quaternions
%
% dual_quat = multiplyDualQuat(dq_left, dq_right)
% Returns the dual quaternion product between two dual quaternions that are 
% stored in the dual quaternion structure, which has members rot and disp, 
% and acts as a chained transformation.
%
% dual_quat = the dual quaternion stacked [0;q_vec]
%
% dq_left = left dual quaternion stacked [0;q_vec]
% dq_right = the right dual quaternion stacked [0;q_vec]
%
% Noah Chapman
% 10883958
% MEGN544
% 10/11/2025
function dual_quat = multiplyDualQuat(dq_left, dq_right)
    r1 = dq_left.rot;
    d1 = dq_left.disp;
    r2 = dq_right.rot;
    d2 = dq_right.disp;

    r = multiplyQuat(r1, r2);
    d = multiplyQuat(r1, d2) + multiplyQuat(d1, r2);

    dual_quat.rot  = r;
    dual_quat.disp = d;
end