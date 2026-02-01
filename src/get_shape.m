function poses = get_shape(shape)

    wall_x = 10;
    center_y = 0;
    center_z = 2.0;
    num_points = 50;

    switch lower(shape)
        case 'line'
           poses = [ [9.8; -3; 0], [9.8; 5; 0] ];
        case 'square'
            scale = 5;
            poses =  [ [wall_x; 0; 0], ... 
                    [wall_x;  scale/2; 0], ...
                    [wall_x;  scale/2; scale], ...
                    [wall_x; -scale/2; scale], ... 
                    [wall_x;  -scale/2; 0], ...
                    [wall_x; 0; 0] ];
        case 'triangle'
            scale = 5;
            poses = [
                      [wall_x; 0; 0], ...
                      [wall_x; scale/2; 0], ...
                      [wall_x; 0; scale], ...
                      [wall_x; -scale/2; 0], ...
                      [wall_x; 0; 0], ...
                    ];
        case 'circle'
            radius = 3.0;
            theta = linspace(0, 2 * pi, num_points);
            y = center_y + radius * cos(theta);
            z = 1.5 + radius * sin(theta);
            x = repmat(wall_x, 1, num_points);
            poses = [x; y; z];
         case 'cosine'
            % Cosine Wave
            t = linspace(-pi, pi, num_points);
            scale = 2.0;
            
            y = center_y + (t * 1.5);
            z = center_z + scale * cos(t);
            x = repmat(wall_x, 1, num_points);
            
            poses = [x; y; z];

        case 'heart'
            % Parametric Heart Equation
            % Note: Raw equations output large numbers (~16), so we scale by 0.15
            t = linspace(0, 2*pi, num_points);
            scale = 0.15; 
            
            h_x = 16 * sin(t).^3;
            h_y = 13 * cos(t) - 5 * cos(2*t) - 2 * cos(3*t) - cos(4*t);
            
            y = center_y + (h_x * scale); 
            z = center_z + (h_y * scale);
            x = repmat(wall_x, 1, num_points);
            
            poses = [x; y; z];

        case 'spiral'
            % Archimedean Spiral: Radius grows as angle grows
            rotations = 3;
            t = linspace(0, rotations*2*pi, num_points);
            
            r = 0.2 * t; % Growth factor
            
            y = center_y + r .* cos(t);
            z = center_z + r .* sin(t);
            x = repmat(wall_x, 1, num_points);
            
            poses = [x; y; z];

        otherwise
            error('Shape not recognized. Options: line, square, circle, cosine, heart, spiral');
    end

end