% calCubicCoeffs: calculate coefficients for zero-velocity cubic
% interpolation between two points
% 
% [a0, a1, a2, a3] = calCubicCoeffs(p0, pf, t0, tf)
% Takes in two (1x1) points and calculates coefficients for zero-velocity
% cubic interpolation between them in given time range, returning those as a vector
% 
% a0 = coefficient for the t^0 term of the cubic 
% a1 = coefficient for the t^1 term of the cubic
% a2 = coefficient for the t^2 term of the cubic
% a3 = coefficient for the t^3 term of the cubic
% 
% p0 = given initial position 
% pf = given final position 
% t0 = given initial time
% tf = given final time
% 
% Noah Chapman
% 10883958
% MEGN 544
% 2025-10-14
function [a0, a1, a2, a3] = calCubicCoeffs(p0, pf, t0, tf)

    t_delta = tf - t0;
    p_delta = pf - p0;

    a0 = p0;
    a1 = 0;
    a2 = (3/(t_delta^2))*p_delta;
    a3 = (-2/(t_delta^3))*p_delta;

end