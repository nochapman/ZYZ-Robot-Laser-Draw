% abbInvKine: Inverse kinematics for ABB robotic arm
%
% [th1, th2, th3, th4, th5, th6, reachable] = abbInvKine(T_des, th_last)
%
% Returns the joint angles required to reach the desired transformation
% for the ABB arm. Handles joint limits, rotation degeneracy, and 
% unreachable states gracefully.
%
% th1-th6: 6 joint angles according to the robot's encoders. If th_last is 
% provided: real scalar values. If th_last is not provided: 8x1 vectors 
% reachable: true if transform can be achieved, false otherwise
%
% T_des: desired homogeneous transform (4x4 matrix)
% th_last: 6x1 vector of last joint angles used to select specific solution.
% If not provided, returns all 8 possible solutions
%
% Author: Noah Chapman
% Student Number: 10883958
% Course: MENG544
% Date: 11/9/2025
function [th1, th2, th3, th4, th5, th6, reachable] = abbInvKine(T_des, th_last)
    a = [0, 0.270, 0.070, 0, 0, 0]';
    d = [0.290, 0, 0, 0.302, 0, 0.072]';
    alpha = [-pi/2, 0, -pi/2, pi/2, -pi/2, 0]';
    theta_offset = [0, pi/2, 0, 0, 0, 0]';

    z_06 = T_des(1:3, 3);
    d_06 = T_des(1:3, 4);
    R_06 = T_des(1:3, 1:3);

    d_05 = d_06 - z_06 * d(6);

    %  Theta 1 candidates 
    theta1_candidates = [atan2(d_05(2), d_05(1)), wrapToPi(atan2(d_05(2), d_05(1)) + pi)];
    if exist('th_last', 'var')
        theta1_candidates(1) = wrap_round(th_last, 1, theta1_candidates(1));
        theta1_candidates(2) = wrap_round(th_last, 1, theta1_candidates(2));
    end

    solutions = zeros(6, 8);
    sol_idx = 0;

    %  Enumerate branches: i (theta1), j (theta3), k (theta5) 
    for i = 1:2
        theta_1 = theta1_candidates(i);
        R_01 = rotZ(theta_1) * rotX(alpha(1));
        d_15 = R_01' * (d_05 - [0; 0; d(1)]);

        % Theta 3 closed-form (two branches)
        delta = atan2(d(4), a(3));
        rho = norm(d_15);
        s = sqrt(d(4)^2 + a(3)^2);
        num = 2*a(2)*s - a(2)^2 - d(4)^2 - a(3)^2 + rho^2;
        den = 2*a(2)*s + a(2)^2 + d(4)^2 + a(3)^2 - rho^2;
        inner_frac = num / den;

        % Guard against tiny negatives due to numerical error
        if inner_frac < 0
            inner_frac = abs(inner_frac);
        end
        phi = 2 * atan(sqrt(inner_frac));
        theta3_candidates = [wrapToPi(pi + phi - delta), wrapToPi(pi - phi - delta)];
        if exist('th_last', 'var')
            theta3_candidates(1) = wrap_round(th_last, 3, theta3_candidates(1));
            theta3_candidates(2) = wrap_round(th_last, 3, theta3_candidates(2));
        end

        for j = 1:2
            theta_3 = theta3_candidates(j);

            % Theta 2 via planar geometry
            p15_x = d_15(1); p15_y = d_15(2);
            alpha_th2 = a(2) + a(3)*cos(theta_3) + d(4)*cos(theta_3 + pi/2);
            beta_th2  = a(3)*sin(theta_3) + d(4)*sin(theta_3 + pi/2);
            sin_num = alpha_th2 * p15_y - beta_th2 * p15_x;
            cos_num = alpha_th2 * p15_x + beta_th2 * p15_y;
            theta_2 = wrapToPi(atan2(sin_num, cos_num) + theta_offset(2));
            if exist('th_last', 'var')
                theta_2 = wrap_round(th_last, 2, theta_2);
            end

            % Orientation part
            R_12 = rotZ(theta_2 - theta_offset(2)) * rotX(alpha(2));
            R_23 = rotZ(theta_3) * rotX(alpha(3));
            R_03 = R_01 * R_12 * R_23;
            R_36 = R_03' * R_06;

            r31 = R_36(3,1); r32 = R_36(3,2); r33 = R_36(3,3);
            r12 = R_36(1,2); r13 = R_36(1,3);
            r22 = R_36(2,2); r23 = R_36(2,3);

            % Theta 5 candidates (wrist pitch)
            theta5_candidates = [atan2(sqrt(r31^2 + r32^2), r33), atan2(-sqrt(r31^2 + r32^2), r33)];
            if exist('th_last', 'var')
                theta5_candidates(1) = wrap_round(th_last, 5, theta5_candidates(1));
                theta5_candidates(2) = wrap_round(th_last, 5, theta5_candidates(2));
            end

            for k = 1:2
                theta_5 = theta5_candidates(k);

                % Theta 4,6 depend on sin(theta_5)
                if abs(sin(theta_5)) < 1e-6
                    % Degenerate wrist: solve theta4+theta6 via convex helper
                    if exist('th_last', 'var')
                        th4_last = th_last(4); th6_last = th_last(6);
                    else
                        th4_last = 0; th6_last = 0;
                    end
                    theta46 = atan2(-r12, r22);
                    [theta_4, theta_6] = convex_opt_soln(th4_last, th6_last, r33, theta46);
                else
                    theta_6 = atan2(-r32/sin(theta_5), r31/sin(theta_5));
                    theta_4 = atan2(-r23/sin(theta_5), -r13/sin(theta_5));
                end

                if exist('th_last', 'var')
                    theta_4 = wrap_round(th_last, 4, theta_4);
                    theta_6 = wrap_round(th_last, 6, theta_6);
                end

                % Store candidate
                sol_idx = sol_idx + 1;
                solutions(:, sol_idx) = [theta_1; theta_2; theta_3; theta_4; theta_5; theta_6];
            end
        end
    end

    %  Filter invalid and trim 
    valid_mask = all(isfinite(real(solutions)), 1) & all(imag(solutions) == 0, 1);
    solutions = real(solutions(:, valid_mask));
    if isempty(solutions)
        % Fallback if no solutions: closest to th_last or zeros
        if exist('th_last', 'var')
            chosen = double(th_last(:));
        else
            chosen = zeros(6,1);
        end
        th_vec = chosen;
        reachable = false;
    else
        % Choose closest solution if th_last is provided, else return all
        if exist('th_last', 'var')
            diffs = solutions - th_last(:);
            [~, best_idx] = min(vecnorm(diffs, 2, 1));
            th_vec = solutions(:, best_idx);
        else
            th_vec = solutions; % 6xN (N<=8)
        end

        % Reachability check (use first or chosen solution)
        if size(th_vec, 2) == 1
            T_sol = dhTransform(a(1), d(1), alpha(1), th_vec(1)) * ...
                    dhTransform(a(2), 0, alpha(2), th_vec(2) - theta_offset(2)) * ...
                    dhTransform(a(3), 0, alpha(3), th_vec(3)) * ...
                    dhTransform(0, d(4), alpha(4), th_vec(4)) * ...
                    dhTransform(0, 0, alpha(5), th_vec(5)) * ...
                    dhTransform(0, d(6), alpha(6), th_vec(6));
        else
            T_sol = dhTransform(a(1), d(1), alpha(1), th_vec(1,1)) * ...
                    dhTransform(a(2), 0, alpha(2), th_vec(2,1) - theta_offset(2)) * ...
                    dhTransform(a(3), 0, alpha(3), th_vec(3,1)) * ...
                    dhTransform(0, d(4), alpha(4), th_vec(4,1)) * ...
                    dhTransform(0, 0, alpha(5), th_vec(5,1)) * ...
                    dhTransform(0, d(6), alpha(6), th_vec(6,1));
        end
        pose_error = norm(T_sol - T_des, 'fro');
        reachable = pose_error < 1e-6;
    end

    if exist('th_last', 'var') || size(th_vec, 2) == 1
        th1 = th_vec(1); th2 = th_vec(2); th3 = th_vec(3);
        th4 = th_vec(4); th5 = th_vec(5); th6 = th_vec(6);
    else
        th1 = th_vec(1, :)'; th2 = th_vec(2, :)'; th3 = th_vec(3, :)';
        th4 = th_vec(4, :)'; th5 = th_vec(5, :)'; th6 = th_vec(6, :)';
    end
end

function value = wrap_round(th_last, idx, input_val)
    % Wrap input_val to be pi-close to th_last(idx)
    value = input_val + pi * round((th_last(idx) - input_val) / pi);
end

function [theta_4, theta_6] = convex_opt_soln(th4_last, th6_last, r33, theta46)
    % Resolve theta_4 and theta_6 in the wrist singular case (sin(theta_5) ~ 0)
    if abs(r33 - 1) < 1e-12
        sgn = 1;
    else
        sgn = -1;
    end
    A = [2, 0, 1;
         0, 2, sgn;
         1, sgn, 0];
    b = [2 * th4_last;
         2 * th6_last;
         theta46];
    x = A \ b;
    theta_4 = x(1);
    theta_6 = x(2);
end
