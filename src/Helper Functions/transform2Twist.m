% transform2Twist(t): Returns the twist vector corresponding to the
% provided homogeneous transform matrix
%
% t = transform2Twist(H)
% Returns the twist vector corresponding to the provided homogeneous 
% transform matrix. The twist should be stacked [v;w th].
%
% t = the twist vector
%
% H = the homogeneous transformation matrix
%
% Noah Chapman
% 10883958
% MEGN544
% 10/11/2025
function t = transform2Twist(H)
    R = H(1:3, 1:3);
    d = H(1:3, 4);

    Omega = rot2AngleAxis(R);
    theta = norm(Omega);
    k = Omega / theta;
    if theta <=0
        t = [d;0;0;0];
        return
    end
    v = (((sin(theta))/(2*(1-cos(theta))) )*eye(3) + (((2*(1-cos(theta)) - theta*sin(theta)))/(2*theta*(1-cos(theta))))*k*transpose(k) - .5*cpMap(k))*d;
    t = [v;k*theta];
end