% twist2Transform(t): Returns the homogeneous transformation matrix 
% corresponding to a 6 element twist vector
%
% H = twist2Transform(t)
% Returns the homogeneous transformation matrix corresponding to a 6 
% element twist vector. The twist should be stacked [v;w th].
%
% H = the homogeneous transformation matrix
%
% t = the twist vector
%
% Noah Chapman
% 10883958
% MEGN544
% 10/11/2025
function H = twist2Transform(t)
    Omega = t(4:6);
    v = t(1:3);
    
    theta = norm(Omega);
    k = (1/theta) * Omega;
    
    if theta <= 0
        R = eye(3);
        d = v;
    else
        R = angleAxis2Rot(theta*k);
        d = (eye(3) - R) * cpMap(k) * v + (k*k.')*v*theta;
    end
    
    
    H = [R, d; 0 0 0 1];
end
