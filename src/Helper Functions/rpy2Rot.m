% rpy2Rot: Returns a rotation matrix corresponding to a roll, pitch & yaw 
% encoded rotation. 
%
% [R_zyx] = rpy2Rot(yaw, pitch, roll) 
% Returns a rotation matrix corresponding to a roll, pitch & yaw 
% encoded rotation. Note RPY is defined as the set of 
% orthogonal rotations rotZ(yaw)rotY(pitch)rotX(roll)
%
% R_zyx = Combined rotation matrix representing a ZYX rotation
%
% yaw = rotation angle about z-axis
% pitch = rotation angle about y-axis
% roll = rotation angle about x-axis
%
% Noah Chapman
% 10883958
% MEGN544
% 10/11/2025
function R_zyx = rpy2Rot(roll, pitch, yaw)
    R_zyx = rotZ(yaw) * rotY(pitch) * rotX(roll);
end