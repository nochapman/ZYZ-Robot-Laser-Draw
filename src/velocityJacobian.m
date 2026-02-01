% velocityJacobian: Conbstructs the velocity Jacobian of a given manipulator
%
% [Jv, JvDot] = velocityJacobian(linkList, paramList, paramRateList)
% Returns the velocity Jacobian of the manipulator given an array of links 
% created by createLink function and the current joint variables.
%
% Jv = velocity Jacobian
% JvDot = time derivative of the velocity Jacobian
%
% linkList = list of joint parameters created by createLink
% paramList = current θ or d values for the joints, according to the
% robot’s encoders (N × 1 array)
% paramRateList = the current thetadot and ddot values for the joints
% (N × 1 array). If paramRateList is not provided check with
% exist(‘paramRateList’,’var’), then return [] for JvDot, otherwise
% calculate JvDot as well.
%
% Noah Chapman
% 10883958
% MEGN544
% 11/20/2025
function [Jv, JvDot] = velocityJacobian(linkList, paramList, paramRateList)
T = eye(4);
N = length(linkList);

d_list = zeros(3, N);
z_list = zeros(3, N);
w_list = zeros(3, N+1);
ddot_list = zeros(3, N);

w_total = [0;0;0];
v_total = [0;0;0];

for i = 1:N
    link = linkList(i);
    if exist('paramRateList', 'var')
        q_dot = paramRateList(i);
    else
        q_dot = 0;
    end
    d_list(:,i) = T(1:3, 4);
    z_list(:,i) = T(1:3, 3);
    if link.isRotary == 1
        w_total = w_total + q_dot*z_list(:,i);
    end
    w_list(:,i) = w_total;
    T_this = dhFwdKine(link, paramList(i));
    v_total = v_total + cross(w_total, T(1:3, 1:3) * T_this(1:3, 4));
    if link.isRotary == 0
        v_total = v_total + (q_dot * z_list(:, i));
    end
    ddot_list(:,i) = v_total;
    T = T*T_this;
end

Jv = zeros(6,N);
JvDot = zeros(6, N);
d_total = T(1:3,4);

for i = 1:N
    link = linkList(i);

    if link.isRotary == 1
        Jv(:,i) = [cross(z_list(:,i),(d_total-d_list(:,i))); z_list(:,i)];
    else
        Jv(:,i) = [z_list(:,i); zeros(3,1)];
    end

    if exist('paramRateList', 'var')
        if i == 1
            w_i = [0;0;0]; v_i = [0;0;0];
        else
            w_i = w_list(:,i-1); v_i = ddot_list(:, i-1);
        end
        if link.isRotary
            JvDot(:,i) = [cross(cross(w_i, z_list(:,i)), (d_total-d_list(:,i))) + cross(z_list(:,i), v_total-v_i); cross(w_i, z_list(:,i)) ];
        else
            JvDot(:,i) = [cross(w_i, z_list(:,i)); zeros(3,1)];
        end
    else
        JvDot = [];
    end
end
