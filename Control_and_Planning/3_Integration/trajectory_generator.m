function [ desired_state ] = trajectory_generator(t, qn, map, path)
% TRAJECTORY_GENERATOR: Turn a Dijkstra or A* path into a trajectory
%
% NOTE: This function would be called with variable number of input
% arguments. In init_script, it will be called with arguments
% trajectory_generator([], [], map, path) and later, in test_trajectory,
% it will be called with only t and qn as arguments, so your code should
% be able to handle that. This can be done by checking the number of
% arguments to the function using the "nargin" variable, check the
% MATLAB documentation for more information.
%
% map: The map structure returned by your load_map function
% path: This is the path returned by your planner (dijkstra function)
%
% desired_state: Contains all the information that is passed to the
% controller, as in phase 2
%
% It is suggested to use "persistent" variables to store map and path
% during the initialization call of trajectory_generator, e.g.
% persistent map0 path0
% map0 = map;
% path0 = path;

desired_state = [];

% use the "persistent" keyword to keep your trajectory around
% inbetween function calls
persistent Map waypoints N segment_dir T_total T_segments alpha;

if nargin == 4
    Map = map;
    waypoints = prune(path, map);
    waypoints = dense(waypoints);
    waypoints = prune(waypoints, map);
    waypoints = merge(waypoints);
    
    % Specify time for each segment
    N = size(waypoints, 2) - 1;
    segment_vec = diff(waypoints, 1, 2);
    segment_lengths = sqrt(sum(segment_vec .* segment_vec));
    segment_dir = segment_vec ./ segment_lengths;
    total_length = sum(segment_lengths);
    
    avg_angle = assess_turnings(waypoints);
    velocity = 4.4 / (1 + avg_angle * 4 / pi);

    T_total = total_length / velocity;
    T_segments = T_total * sqrt(segment_lengths) / sum(sqrt(segment_lengths));
        
    % =================== Piece-wise 3rd order time polynomials ===================
    alpha = zeros(4, N);
    
    for i = 1 : N
        T = T_segments(i);

        A = [1, 0, 0, 0;
             1, T, T^2, T^3;
             0, 1, 0, 0;
             0, 1, 2*T, 3*T^2];
        b = [0, segment_lengths(i), 0, 0]';
        alpha(:, i) = A \ b;
    end

else
    
    t_index = 1;
    for i = 1 : N
        if (t < sum(T_segments(1:i)) && t >= sum(T_segments(1:i-1)))
            t_index = i;
            break;
        end
    end
    
    t_current = t - sum(T_segments(1:t_index-1));
    
    t_polynomial_pos = [1, t_current, t_current ^ 2, t_current ^ 3];
    t_polynomial_vel = [0, 1, 2 * t_current, 3 * t_current ^ 2];
    t_polynomial_acc = [0, 0, 2, 6 * t_current];
    
    if t == 0
        pos = waypoints(:, 1);
        vel = [0; 0; 0];
        acc = [0; 0; 0];
        yaw = 0;
        yawdot = 0;
    else
        if t < T_total
            pos = waypoints(:, t_index) + segment_dir(:, t_index) * t_polynomial_pos * alpha(:, t_index);
            vel = segment_dir(:, t_index) * t_polynomial_vel * alpha(:, t_index);
            acc = segment_dir(:, t_index) * t_polynomial_acc * alpha(:, t_index);
            yaw = 0;
            yawdot = 0;
        else
            pos = waypoints(:, end);
            vel = [0; 0; 0];
            acc = [0; 0; 0];
            yaw = 0;
            yawdot = 0;
        end
    end

desired_state.pos = pos(:);
desired_state.vel = vel(:);
desired_state.acc = acc(:);
desired_state.yaw = yaw;
desired_state.yawdot = yawdot;

end
end

function waypoints = nudge(waypoints, map)
% waypoints here should have dimension 3 * N
    N = size(waypoints, 2);
    
    for i = 2 : N - 1
        vec_1 = waypoints(:, i - 1) - waypoints(:, i);
        vec_2 = waypoints(:, i + 1) - waypoints(:, i);
        dir = 0.5 * (vec_1 + vec_2);
        dir = dir / norm(dir);
        
        for k = 1 : 1000
            nudge_pts = waypoints(:, i) + k * 0.02 * dir;
            if (detect_intersection(nudge_pts', waypoints(:, i - 1)', map) || ...
                detect_intersection(nudge_pts', waypoints(:, i + 1)', map))
                waypoints(:, i) = nudge_pts - 0.02 * dir;
                break;
            end
        end
    end
end

function waypoints = merge(path)
% path should have dimension N * 3

    % Define a threshold
    epsilon = 0.001;
    
    % Number of points from the dijkstra planning
    N = size(path, 1);
    
    % Starting point
    waypoints = path(1, :)';
    
    % Vectors formed by consecutive points
    segment_dir = diff(path, 1, 1);
    
    for i = 2 : N - 1
        v1 = segment_dir(i - 1, :);
        v2 = segment_dir(i, :);
        
        if abs(v1 * v2' / (norm(v1) * norm(v2)) - 1) > epsilon
            waypoints = [waypoints, path(i, :)'];
        end
    end
    
    % Append the goal point
    waypoints = [waypoints, path(end, :)'];
end

function waypoints = prune(path, map)
% path should have dimension N * 3; N here is not the number of segments
% but end points
    N = size(path, 1);
    frontier = Inf(1, N);
    frontier(1) = 0;
    parents = 1: N;
    
    for i = 2 : N
        segment_end = path(i, :);
        for j = i - 1 : -1 : 1
            segment_start = path(j, :);
            collision = detect_intersection(segment_start, segment_end, map);
            if ~collision
                if norm(segment_start - segment_end) + frontier(j) < frontier(i)
                    frontier(i) = norm(segment_start - segment_end) + frontier(j);
                    parents(i) = j;
                end
            end
        end
    end
    
    last_ind = N;
    waypoints = path(last_ind, :);
    
    while last_ind ~= 1
        last_ind = parents(last_ind);
        waypoints = [path(last_ind, :); waypoints];
    end
            

end

function flag = detect_intersection(segment_start, segment_end, map)
% End points should have dimension 1 * 3

    flag = false;
    
    length = norm(segment_end - segment_start);
    dir = (segment_end - segment_start) / length;
    
    XYZs = segment_start + (0:0.002:length-0.002)' * dir;
    
    collision = map.collide(XYZs);
    
    if sum(collision) > 0
        flag = true;
    end
    
end

function avg_angle = assess_turnings(waypoints)
% waypoints here should have dimension 3 * N
    N = size(waypoints, 2);
    
    angles = zeros(N - 2, 1);
    
    for i = 2 : N - 1
        vec_1 = waypoints(:, i) - waypoints(:, i - 1);
        vec_2 = waypoints(:, i + 1) - waypoints(:, i);
        angles(i - 1) = acos(vec_1' * vec_2 / (norm(vec_1) * norm(vec_2)));
    end
    
    avg_angle = mean(angles);
        
end

function dense_waypoints = dense(waypoints)
% waypoints here should have dimension N * 3
    N = size(waypoints, 1);
    
    dense_waypoints = waypoints(1, :);
    
    % N - 1 segments to make dense
    for i = 1 : N - 1
        start_pt = waypoints(i, :);
        end_pt = waypoints(i + 1, :);
        length = norm(start_pt - end_pt);
        dir = (end_pt - start_pt) / length;
        
        for j = 0.2 : 0.2 : length - 0.2
            dense_waypoints = [dense_waypoints; start_pt + j * dir];
        end
        dense_waypoints = [dense_waypoints; end_pt];
    end
end
