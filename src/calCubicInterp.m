% calCubicInterp: calculate interpolation between arbitrary vectors given
% cubic interpolation coefficients. 
% 
% V_evals = calCubicInterp(v0, vf, a0, a1, a2, a3, t0, T_evals)
% We begin with the given initial vector, and end with the final vector, 
% interpolating based on the given coefficients in between, evaluated at the 
% listed times
% 
% V_evals = list of interpolated vectors, in same order as given list of
% evaluation times
% 
% v0 = given initial vector 
% vf = given final vector
% a0 = given coefficient for the t^0 term of the cubic 
% a1 = given coefficient for the t^1 term of the cubic
% a2 = given coefficient for the t^2 term of the cubic
% a3 = given coefficient for the t^3 term of the cubic
% t0 = given initial time
% T_evals = given list of times to evaluation interpolation at
% 
% Noah Chapman
% 10883958
% MEGN 544
% 2025-10-14
function [V_evals] = calCubicInterp(v0, vf, a0, a1, a2, a3, t0, T_evals)

    cubic_func = @(t) a0 + a1.*t + a2.*(t.^2) + a3.*(t.^3);
    %fplot(cubic_func, [t0, T_evals(end)])

    v_sz = size(v0);
    v_len = v_sz(1);
    t_sz = size(T_evals);
    t_len = t_sz(1);

    V_evals = zeros(v_len, t_len);

    v_delta = vf - v0;

    for i = 1:1:v_len
        for j = 1:1:t_len

            current_t = T_evals(j);
            interp_scalar = cubic_func(current_t-t0);

            initial = v0(i);
            offset = v_delta(i)*interp_scalar;
            V_evals(i,j) = initial + offset;

        end

        %V_evals

end