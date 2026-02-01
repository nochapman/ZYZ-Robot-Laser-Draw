function poses = get_image_shape(image_file)
    raw_img = imread(image_file);

    % To grey scale
    if size(raw_img, 3) == 3
        gray_img = rgb2gray(raw_img);
    else
        gray_img = raw_img;
    end
    bw_edges = edge(gray_img, 'Canny');
    [rows, cols] = find(bw_edges);

    if isempty(rows)
        error('No edges found in the image!');
    end
    raw_points = [cols, rows]; % N x 2 matrix
    N = size(raw_points, 1);

    ordered_points = zeros(N, 2);
    visited = false(N, 1);

    current_idx = 1;
    visited(1) = true;
    ordered_points(1, :) = raw_points(1, :);
    
    for k = 2:N
        current_pt = raw_points(current_idx, :);
        
        % Calculate distances from current point to all UNVISITED points
        dists = sum((raw_points - current_pt).^2, 2);
        dists(visited) = inf; % Ignore visited points
        
        % Find the closest neighbor
        [min_dist, next_idx] = min(dists);
        
        % Check for "Jumps" (Pen Lift)
        % If the next closest point is very far, we are starting a new shape.
        % The robot will drag the laser across, which is fine for now.
        
        if isinf(min_dist)
            break; % Should not happen unless logic fails
        end
        
        visited(next_idx) = true;
        ordered_points(k, :) = raw_points(next_idx, :);
        current_idx = next_idx;
    end
    
    % --- 4. Downsampling ---
    % Images are dense! We don't need every pixel. 
    % Take every 5th point (adjust this based on your simulation speed)
    downsample_rate = 10; 
    final_2d_points = ordered_points(1:downsample_rate:end, :);
    
    % --- 5. Coordinate Mapping (Scaling to Wall) ---
    % Wall Settings (match your get_shape.m)
    wall_x = 10;
    center_y = 0;
    center_z = 2;
    
    % Normalize image coordinates to range [-1, 1]
    % Note: Image (0,0) is Top-Left. Robot Z grows Up. We must flip Y-axis (rows).
    img_width = size(bw_edges, 2);
    img_height = size(bw_edges, 1);
    
    u = final_2d_points(:, 1); % Columns
    v = final_2d_points(:, 2); % Rows
    
    % Scale factors (Make the drawing 4 meters wide max)
    scale_factor = 6.0; 
    
    % Map Image Cols (u) -> Robot Y
    % (u - width/2) centers it
    y_traj = center_y + ((u - img_width/2) / img_width) * scale_factor;
    
    % Map Image Rows (v) -> Robot Z 
    % Note the negative sign to flip image (row 0 is top, Z+ is up)
    z_traj = center_z - ((v - img_height/2) / img_height) * scale_factor;
    
    x_traj = repmat(wall_x, length(y_traj), 1);
    
    % Output 3xN format
    poses = [x_traj, y_traj, z_traj]';
end