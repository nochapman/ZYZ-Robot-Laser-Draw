% constAccelInterp: performs constant acceleration-constant velocity
% interpolation over a given set of waypoints and a sample time
% 
% [p,v,a] = constAccelInterp(t, trajectory, transPercent)
% takes in a sampling time, a set of waypoints with associated
% target times and the percent between waypoints to swtich from constant
% acceleration to constant velocity, and then calculates the position,
% velocity and acceleration at the sampling times
% 
% p = vector of interpolated positions at sample times
% v = vector of interpolated velocities at sample times
% a = vector of interpolated accelerations at sample times
% 
% t = sampling time
% trajectory = matrix of waypoints, first index of each waypoint column
% should be the target time, remaining indexes should be waypoint values
% transPercent = percent of distance between waypoints to swtich from
% constant acceleration to constant velocity
% 
% Noah Chapman
% 10883958
% MEGN 544
% 2025-11-06
function [p,v,a] = constAccelInterp(t, trajectory, transPercent)

    first_time = trajectory(1,1);
    last_time = trajectory(end,1);

    % clamp t to within bounds
    if t < first_time
        t = first_time;
    end
    if t > last_time
        t = last_time;
    end

    waypoint_count = size(trajectory, 1);
    waypoint_times = trajectory(:,1);
    waypoint_poses = trajectory(:,2:end);

    % first we determine which segment we are in
    last_waypoint_index = 0;
    next_waypoint_index = 0;

    % catch that funny edge case for the very end of the path
    if t == waypoint_times(waypoint_count)
        p = waypoint_poses(waypoint_count, :);
        v = zeros(1, size(waypoint_poses,2));
        a = zeros(1, size(waypoint_poses,2));
        return
    end

    % special-case: if we're at or before the very first waypoint, return
    % the first waypoint pose with zero velocity/acceleration. This avoids
    % relying on a "previous" segment velocity that doesn't exist and
    % prevents undefined behaviour at the start of the trajectory.
    if t <= waypoint_times(1)
        p = waypoint_poses(1, :);
        v = zeros(1, size(waypoint_poses,2));
        a = zeros(1, size(waypoint_poses,2));
        return
    end
        
    % iterate only to waypoint_count-1 to avoid accessing i+1 out-of-bounds
    for i = 1:1:(waypoint_count-1)
        if t >= waypoint_times(i) && t < waypoint_times(i+1)
           last_waypoint_index = i;
           next_waypoint_index = i+1;
           break
        end
    end

    % safety fallback: if we somehow didn't find a segment, clamp to first
    % valid segment to avoid indexing at 0
    if last_waypoint_index == 0
        last_waypoint_index = 1;
        next_waypoint_index = min(2, waypoint_count);
    end
    
    

    % then we determine our transitioning time (tau)
    current_segment_time = waypoint_times(next_waypoint_index) - waypoint_times(last_waypoint_index);
    next_segment_time = 10e10;
    % quick special-case handling if we don't have a next segment due to
    % being at the end of the path
    if next_waypoint_index < waypoint_count
        next_segment_time = waypoint_times(next_waypoint_index+1) - waypoint_times(next_waypoint_index);
    end
    transitioning_time = transPercent*min(current_segment_time, next_segment_time);
    const_vel_start_time = waypoint_times(last_waypoint_index) + transitioning_time;
    const_vel_stop_time = waypoint_times(next_waypoint_index) - transitioning_time;

    % now can start calculating velocities and such
    current_segment_delta = waypoint_poses(next_waypoint_index, :) - waypoint_poses(last_waypoint_index, :);
    current_segment_vel = current_segment_delta/(waypoint_times(next_waypoint_index) - waypoint_times(last_waypoint_index));

    % if we are in a constant-velocity segment, our job is done
    if t > const_vel_start_time && t < const_vel_stop_time
        p = waypoint_poses(last_waypoint_index,:) + current_segment_vel*(t-waypoint_times(last_waypoint_index));
        v = current_segment_vel;
        a = zeros(size(v));
    % if not, we have more work to do
    % first we need to know which constant acceleration section we're in
    % either we are approaching the constant acceleration section, or we
    % are leaving it
    elseif t <= const_vel_start_time % approaching constant velocity section
        % first we must find the velocity of the previous constant velocity
        % section in order to find our acceleration
        prev_segment_vel = zeros(1,size(waypoint_poses,2));
        if last_waypoint_index > 1
            prev_segment_delta = waypoint_poses(last_waypoint_index, :) - waypoint_poses(last_waypoint_index-1, :);
            prev_segment_vel = prev_segment_delta/(waypoint_times(last_waypoint_index) - waypoint_times(last_waypoint_index-1));
        end
        
        last_transition_point = waypoint_poses(last_waypoint_index, :) - prev_segment_vel*transitioning_time;

        time_delta = (t - waypoint_times(last_waypoint_index)+transitioning_time);
        
        a = (current_segment_vel - prev_segment_vel)/(2*transitioning_time);
        v = prev_segment_vel + a * time_delta;
        p = last_transition_point + (prev_segment_vel*time_delta) + (0.5*a*(time_delta^2));

    elseif t >= const_vel_stop_time % leaving constant velocity section
        % first we must find the velocity of the next constant velocity
        % section in order to find our acceleration 
        next_segment_vel = zeros(1,size(waypoint_poses,2));
        if next_waypoint_index < waypoint_count
            next_segment_delta = waypoint_poses(next_waypoint_index+1, :) - waypoint_poses(next_waypoint_index, :);
            next_segment_vel = next_segment_delta/(waypoint_times(next_waypoint_index+1) - waypoint_times(next_waypoint_index));
        end 

        last_transition_point = waypoint_poses(next_waypoint_index, :) - current_segment_vel*transitioning_time;
        
        time_delta = (t - const_vel_stop_time);

        a = (next_segment_vel - current_segment_vel)/(2*transitioning_time);
        v = current_segment_vel + a*time_delta;
        p = last_transition_point + (current_segment_vel*time_delta) + (0.5*a*(time_delta^2));
    end

end