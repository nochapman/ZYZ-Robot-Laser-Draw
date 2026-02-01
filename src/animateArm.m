function animateArm(ikSolutions, torques)
    % animateArm: Real-time tiled animation with 3D trail effect
    % Noah Chapman
    
    N = size(ikSolutions, 2);
    wall_x = 10; %
    wall_pts = [ [wall_x; -5; -5] [wall_x; 5; -5] [wall_x; 5; 5] [wall_x; -5; 5] ];
    
    dt = 0.05; % Simulation step delay

    % Screen tiling logic
    scrsz = get(0, 'ScreenSize');
    w = scrsz(3)/3; 
    h = scrsz(4)/2;

    % --- Figure 1: 3D Robot Arm & Trail ---
    f1 = figure(1); clf(f1);
    set(f1, 'Name', '3D Kinematics', 'NumberTitle', 'off', 'OuterPosition', [1, scrsz(4)-h, w, h]);
    arm_ax = axes(f1);
    hold(arm_ax, 'on'); grid(arm_ax, 'on'); view(arm_ax, 3);
    axis(arm_ax, [-2 12 -6 6 -2 6]); %
    
    fill3(arm_ax, wall_pts(1,:), wall_pts(2,:), wall_pts(3,:), 'b', 'FaceAlpha', 0.1);
    
    % The "Trail": Plotting a line that will grow over time
    h_3d_trail = plot3(arm_ax, NaN, NaN, NaN, 'r:', 'LineWidth', 2); 
    h_hit = plot3(arm_ax, NaN, NaN, NaN, 'gx', 'MarkerSize', 10, 'LineWidth', 2);
    h_arm = plot3(arm_ax, [0 0 0 0], [0 0 0 0], [0 0 0 0], 'bo-', 'LineWidth', 2, 'MarkerSize', 6);
    h_laser = plot3(arm_ax, [0 0], [0 0], [0 0], 'r-', 'LineWidth', 2);

    % --- Figure 2: Laser Trace (Wall) ---
    f2 = figure(2); clf(f2);
    set(f2, 'Name', 'Wall Trace', 'NumberTitle', 'off', 'OuterPosition', [w, scrsz(4)-h, w, h]);
    wall_ax = axes(f2);
    hold(wall_ax, 'on'); grid(wall_ax, 'on'); axis(wall_ax, 'equal');
    h_trace = plot(wall_ax, NaN, NaN, 'r.-', 'MarkerSize', 4);

    % --- Figure 3: Torque Telemetry ---
    f3 = figure(3); clf(f3);
    set(f3, 'Name', 'Torque (Nm)', 'NumberTitle', 'off', 'OuterPosition', [2*w, scrsz(4)-h, w, h]);
    torque_ax = axes(f3);
    hold(torque_ax, 'on'); grid(torque_ax, 'on');
    h_t1 = plot(torque_ax, 1:N, nan(1,N), 'r', 'DisplayName', 'J1');
    h_t2 = plot(torque_ax, 1:N, nan(1,N), 'g', 'DisplayName', 'J2');
    h_t3 = plot(torque_ax, 1:N, nan(1,N), 'b', 'DisplayName', 'J3');
    legend(torque_ax, 'Location', 'northeast');
    xlim(torque_ax, [1 N]);

    % Pre-allocate buffers for trail data
    trail_x = nan(1, N);
    trail_y = nan(1, N);
    trail_z = nan(1, N);

    % --- Animation Loop ---
    for k = 1:N
        params = ikSolutions(:,k);
        
        % Kinematics (Logic from your drawArm.m)
        T1 = dhTransform(dhTable(1, 'a'), dhTable(1, 'd'), dhTable(1, 'alpha'), params(1));
        T2 = dhTransform(dhTable(2, 'a'), dhTable(2, 'd'), dhTable(2, 'alpha'), params(2));
        T3 = dhTransform(dhTable(3, 'a'), dhTable(3, 'd'), dhTable(3, 'alpha'), params(3));
        T4 = dhTransform(0, params(4), 0, 0);

        H1 = T1; H2 = T1*T2; H3 = T1*T2*T3; H4 = T1*T2*T3*T4;
        pts = [zeros(3,1), H1(1:3,4), H2(1:3,4), H3(1:3,4)];
        p4 = H4(1:3,4); % End-effector position

        % Store Trail Point
        trail_x(k) = p4(1); trail_y(k) = p4(2); trail_z(k) = p4(3);

        % Update 3D Visuals
        h_arm.XData = pts(1,:); h_arm.YData = pts(2,:); h_arm.ZData = pts(3,:);
        h_laser.XData = [pts(1,4) p4(1)]; h_laser.YData = [pts(2,4) p4(2)]; h_laser.ZData = [pts(3,4) p4(3)];
        h_hit.XData = p4(1); h_hit.YData = p4(2); h_hit.ZData = p4(3);
        
        % Update 3D Trail
        h_3d_trail.XData = trail_x(1:k); 
        h_3d_trail.YData = trail_y(1:k); 
        h_3d_trail.ZData = trail_z(1:k);

        % Update Wall Trace & Torques
        h_trace.XData = trail_y(1:k); h_trace.YData = trail_z(1:k);
        h_t1.YData(k) = torques(1, k);
        h_t2.YData(k) = torques(2, k);
        h_t3.YData(k) = torques(3, k);
        
        drawnow; 
        pause(dt); 
    end
end