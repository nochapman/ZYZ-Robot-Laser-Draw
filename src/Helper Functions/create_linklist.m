function linkList = create_linklist()
    linkList = [];
    radius_inner = 0.04;           % Radius of hollow cylinder (meters)
    radius_outer = 0.05;
    density = 2700;          % Density of aluminum (kg/m^3)
    % append i to linklist
    for i = 1:4
        a_i = dhTable(i, 'a');
        d_i = dhTable(i, 'd');
        alpha_i = dhTable(i, 'alpha');

        if i == 4   % It's a laser...no mass or inertia
            link_i = createLink(a_i, [], alpha_i, 0, 0, 0, 0, 0);
        else    % Inertia tensor calculations
            if a_i == 0 % Links 1 and 2: hollow cylinder along z
                link_len = d_i;
                volume = pi * (radius_outer^2 - radius_inner^2) * link_len;
                mass = density * volume;
                Ixx = (1/12) * mass * (3*(radius_outer^2 + radius_inner^2) + link_len^2);
                Iyy = Ixx;
                Izz = 0.5 * mass * (radius_outer^2 + radius_inner^2);
                inertia_tensor = diag([Ixx, Iyy, Izz]);
                cent_of_mass = [0; 0; link_len/2];
            else    % Link 3 along x
                link_len = a_i;
                volume = pi * (radius_outer^2 - radius_inner^2) * link_len;
                mass = density * volume;
                Ixx = 0.5 * mass * (radius_outer^2 + radius_inner^2);
                Iyy = (1/12) * mass * (3*(radius_outer^2 + radius_inner^2) + link_len^2);
                Izz = Iyy;
                inertia_tensor = diag([Ixx, Iyy, Izz]);
                cent_of_mass = [link_len/2; 0; 0];
            end
            link_i = createLink( a_i, d_i, alpha_i, [], 0, cent_of_mass, mass, inertia_tensor );
        end
        
        linkList = [linkList, link_i];
    end
end