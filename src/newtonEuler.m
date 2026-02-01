% newtonEuler calculates the joint torques, to obtain specified motion
% targets (pos, vel, accel)
%
% [jointTorques] = newtonEuler( linkList, paramList,
%                      paramListDot, paramListDDot,  boundry_conditions )
%
% Outputs:
%  jointTorques: list of torques to be exerted by each jonint to acheive
%  targets
%
% Inputs:
%   linkList: list of link structs describing the robot
%   paramList: list of position targets for each joint, in the same order
%   as linklist 
%   paramListDot: list of velocity targets for each joint, in the same order
%   as linklist 
%   paramListDDot: list of acceleration targets for each joint, in the same order
%   as linklist 
%   boundry_conditions: list of boundary conditions known about the world
%   frame's motion and torques/forces exterted on the end effector
%
% Noah Chapman
% 10883958
% MEGN 544
% 2025-12-09
function [jointTorques] = newtonEuler( linkList, paramList, paramListDot, paramListDDot, boundry_conditions )

    % test case 413 has single prismatic joint 
    % 414 is prismatic then rotary 
    % 415 is rotary then rotary
    % test case 416 has single rotary joint
    % 422 is 3-link pure prismatic

    num_joints = length(linkList);
    num_statics = 0;

    jointTorques = zeros(length(paramList),1);

    T_0_i_list = zeros(4,4,num_joints); % transform from world frame to origin Oi
    r_0_i1_i_list = zeros(3, num_joints); % displacement from Oi-1 (frame of joint i) to joint i's center of mass, in frame 0 coordinates
    r_0_ii_list = zeros(3, num_joints); % displacement from Oi (frame of joint i+1) to joint i's center of mass, in frame 0 coordinates
    rddot_0_0i_list = zeros(3, num_joints); % accleration of joint i's center of mass, in frame 0 coordinates
    w0_i_list = zeros(3,num_joints); % angular velocity of frame i in frame 0 coordinates
    alpha_0_i_list = zeros(3,num_joints); % angluar acceleration of frame i in frame 0 coordinates
    a_0_i_list = zeros(3,num_joints); % linear acceleration of frame i in frame 0 coordinates

    % continously updated values used in the loops
    w_last = boundry_conditions.base_angular_velocity;
    alpha_last = boundry_conditions.base_angular_acceleration;
    a_last = boundry_conditions.base_linear_acceleration;
    z_last = [0;0;1];
    T_0_i_running = eye(4);

    for i = 1:1:num_joints
        %fprintf("Starting forward pass iteration: %d\n", i)
        active_parameter_index = i - num_statics;
        q = paramList(active_parameter_index);
        qdot = paramListDot(active_parameter_index);
        qddot = paramListDDot(active_parameter_index);

        current_link = linkList(i);

        % Calculate link transform from i-1 to i
        if current_link.isRotary == 1 || current_link.isRotary == 0
            T_i1_i = dhFwdKine(current_link, q);
        else % if we have a static link, we don't have an active parameter and should keep track for future loops
            T_i1_i = dhFwdKine(current_link);
            num_statics = num_statics + 1;
        end

        % get the stuff we need from the last transform before updating
        R_0_i1 = T_0_i_running(1:3,1:3);
        % then unpdate the running transform from world frame to origin i
        T_0_i_running = T_0_i_running * T_i1_i;
        T_0_i_list(:,:,i) = T_0_i_running;

        % calculate our angular velocity
        w0_i_list(:,i) = w_last;
        if current_link.isRotary == 1 
            w0_i_list(:,i) = w0_i_list(:,i) + qdot * z_last;
        end

        % then our angular acceleration 
        alpha_0_i_list(:,i) = alpha_last;
        if current_link.isRotary == 1 
            alpha_0_i_list(:,i) = alpha_0_i_list(:,i) + qddot * z_last + cross(qdot * w_last, z_last);
        end

        % then our linear acceleration 
        d_0_i1_i = R_0_i1 * T_i1_i(1:3,4);
        a_0_i_list(:,i) = a_last + cross(alpha_0_i_list(:,i), d_0_i1_i) + cross(w0_i_list(:,i), cross(w0_i_list(:,i), d_0_i1_i));
        if current_link.isRotary == 0
            a_0_i_list(:,i) = a_0_i_list(:,i) + cross(2 * qdot * w_last, z_last) + qddot * z_last;
        end

        % then our com displacements from both Oi-1 and Oi
        R_i1_i = T_i1_i(1:3,1:3);
        
        r_0_ii_list(:,i) = R_0_i1 * R_i1_i * current_link.com;
        r_0_i1_i_list(:,i) = d_0_i1_i + r_0_ii_list(:,i);

        % then the acceleration of that center of mass in the 0 frame 
        rddot_0_0i_list(:,i) = a_last + cross(alpha_0_i_list(:,i), r_0_i1_i_list(:,i)) + cross(w0_i_list(:,i), cross(w0_i_list(:,i), r_0_i1_i_list(:,i)));
        if current_link.isRotary == 0
            rddot_0_0i_list(:,i) = rddot_0_0i_list(:,i) + cross(2 * qdot * w_last, z_last) + qddot * z_last;
        end

        % update tracker variables
        w_last(:) = w0_i_list(:,i);
        alpha_last(:) = alpha_0_i_list(:,i);
        a_last(:) = a_0_i_list(:,i);
        z_last(:) = T_0_i_list(1:3,3,i);
        %fprintf("Ending forward pass iteration: %d\n", i)
    end
    
    f_next = T_0_i_running(1:3,1:3) * boundry_conditions.distal_force;
    n_next = T_0_i_running(1:3,1:3) * boundry_conditions.distal_torque;

    for i = num_joints:-1:1
        %fprintf("Starting backward pass iteration: %d\n", i)
        active_parameter_index = i - num_statics;

        current_link = linkList(i);

        % get our inertia tensor in the world frame
        R_0_i = T_0_i_list(1:3,1:3,i);
        I_0_i = R_0_i * current_link.inertia * (R_0_i');

        % calculate net force and torque at joint i
        f_i = current_link.mass * rddot_0_0i_list(:,i);
        n_i = I_0_i * alpha_0_i_list(:,i) + cross(w0_i_list(:,i), I_0_i * w0_i_list(:,i));

        % and then the constraint force between joints
        f_i1_i = f_i + f_next ;
        n_i1_i = n_i + n_next + cross(r_0_i1_i_list(:,i), f_i1_i) - cross(r_0_ii_list(:,i), f_next);
    
        % update tracker variables
        f_next = f_i1_i;
        n_next = n_i1_i;

        % get our z axis sorted
        z_last = [0;0;1];
        if i > 1
            z_last = T_0_i_list(1:3,3,i-1);
        end

        % and calculate the joint torques
        if current_link.isRotary == 1 
            jointTorques(active_parameter_index) = z_last' * n_i1_i;
        elseif current_link.isRotary == 0 
            jointTorques(active_parameter_index) = z_last' * f_i1_i;
        else % static joints cannot apply torque or force
            num_statics = num_statics-1;
        end
        %fprintf("Ending backward pass iteration: %d\n", i)
    end
    
end