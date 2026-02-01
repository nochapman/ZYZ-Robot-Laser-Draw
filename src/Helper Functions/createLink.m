% createLink: Build a DH link with all required robot parameters.
%
% [L] = createLink(a, d, alpha, theta, offset, centOfMass, mass, inertia)
%
% L = structure that describes one robotic link
%
% a = DH parameter a  (meters)
% d = DH parameter d  (meters)
% alpha = DH parameter alpha  (radians)
% theta = joint variable provided by the encoder.
%                Pass [] if the joint angle is not yet known.
% offset = difference between the encoder reading and the DH zero‑angle
%                (theta offset, radians for rotary joints, meters for prismatic).
% centOfMass = position of the link's center of mass in the link frame
%                (3×1 vector, metres)
% mass = link mass (kg)
% inertia = link mass moment of inertia about its own axes
% isRotary = 1 if the joint is rotational, 0 if prismatic, -1 for a fixed
%                (static) link.
%
% L = createLink(0.14, 3.4, pi/3, [], [0;0;0], 2.5, diag([0.01,0.02,0.015]));
%                 
% Author: Noah Chapman
% Student Number: 10883958
% Course: MENG544
% Date: 11/4/2025

function L = createLink(a, d, alpha, theta, offset, centOfMass, mass, inertia)
    
    L.a = a;
    L.d = d;
    L.alpha = alpha;
    L.theta = theta;
    L.offset = offset;
    L.com = centOfMass;
    L.mass = mass;
    L.inertia = inertia;

    if isempty(theta)
        L.isRotary = 1;
    elseif isempty(d)
        L.isRotary = 0;
    else
        L.isRotary = -1;
    end
end
