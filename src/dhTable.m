function value = dhTable(link, column)

    % The table is in the format: [a, alpha, d, theta]
    % (or your preferred convention)
    
    % Define parameters for each joint
    %         [a,   d,   alpha,   theta_offset]
    link_len = 1;
    table = [0,  link_len,    pi/2,   0;
            0,   link_len,   -pi/2,    0  ;
            link_len,     0,       0,    0;
            0,     0,      0,      0];
    
    % Assemble the final table
    if strcmp(column, 'a')
        value = table(link, 1);
    elseif strcmp(column, 'd')
        value = table(link, 2);
    elseif strcmp(column, 'alpha')
        value = table(link, 3);
    elseif strcmp(column, 'offset')
        value = table(link, 4);
    end
end