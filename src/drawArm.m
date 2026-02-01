function [p4, p3] = drawArm(thetas, ax)
    % drawArm: Handles 3 Rotary + 1 Prismatic Link
    % thetas: 4x1 vector [theta1; theta2; theta3; d4]
    
    % 1. Reconstruct Transformations manually to get points
    % (Using your dhTable values: [a, d, alpha, theta])
    
    % Link 1 (Rotary)
    T1 = dhTransform(dhTable(1, 'a'), dhTable(1, 'd'), dhTable(1, 'alpha'), thetas(1));
    
    % Link 2 (Rotary)
    T2 = dhTransform(dhTable(1, 'a'), dhTable(2, 'd'), dhTable(2, 'alpha'), thetas(2));
    
    % Link 3 (Rotary - Wrist)
    T3 = dhTransform(dhTable(3, 'a'), dhTable(3, 'd'), dhTable(3, 'alpha'), thetas(3));
    
    % Link 4 (Prismatic - Laser)
    % a=0, d=thetas(4), alpha=0, theta=0
    T4 = dhTransform(0, thetas(4), 0, 0);

    % Chain them
    H1 = T1;
    H2 = T1 * T2;
    H3 = T1 * T2 * T3;
    H4 = T1 * T2 * T3 * T4;

    % Extract Positions
    p0 = [0;0;0];
    p1 = H1(1:3, 4);
    p2 = H2(1:3, 4);
    p3 = H3(1:3, 4); % Wrist position
    p4 = H4(1:3, 4); % Laser Tip position (on the wall)

    % Plot
    if nargin > 1 && ~isempty(ax)
        cla(ax);
        % Draw Robot Arm (Blue)
        plot3(ax, [p0(1) p1(1) p2(1) p3(1)], ...
                  [p0(2) p1(2) p2(2) p3(2)], ...
                  [p0(3) p1(3) p2(3) p3(3)], 'bo-', 'LineWidth', 2, 'MarkerSize', 6);
        
        hold(ax, 'on');
        % Draw Laser Beam (Red) - From Wrist (p3) to Tip (p4)
        plot3(ax, [p3(1) p4(1)], ...
                  [p3(2) p4(2)], ...
                  [p3(3) p4(3)], 'r-', 'LineWidth', 2);
                  
        xlabel(ax,'X'); ylabel(ax,'Y'); zlabel(ax,'Z');
        axis(ax,[-2 12 -6 6 -2 6]); % Adjusted axis to see the wall at x=10
        grid(ax,'on'); view(ax,3);
        title(ax,'3D Arm with Laser');
    end
end