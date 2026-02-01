% dhFwdKine: Compute forward kinematics using DH parameters.
%
% [H] = dhFwdKine(linkList, paramList)
%
% H 4×4×N array of homogeneous transforms from the base to each link.The 
% last transform (H(:,:,end)) is the end‑effector pose in the base frame.
%
% linkList = 1xN struct array returned by createLink.
% paramList = Nx1 vector of current joint variables read from the encoders.  
% For a rotary joint this is an angle; for a prismatic joint it is a displacement.
%
% Author: Noah Chapman
% Student Number: 10883958
% Course: MENG544
% Date: 11/4/2025
function H = dhFwdKine(linkList, paramList)

    H = eye(4);

    % Loop through each link
    for i = 1:length(linkList)
        L = linkList(i);

        % Apply encoder offsets depending on joint type
        if L.isRotary == 1
            L.theta = paramList(i) - L.offset;
        elseif L.isRotary == 0
            L.d = paramList(i) - L.offset;
        end

        % Compute individual transform
        A = dhTransform(L.a, L.d, L.alpha, L.theta);

        % Multiply into total transform
        H = H * A;
    end
end
