function [desired_state] = trajectory_generator(t, qn, varargin)
% TRAJECTORY_GENERATOR: Turn a Dijkstra or A* path into a trajectory
% t: A scalar, specifying inquiry time
%
% varargin: variable number of input arguments. In the framework,
% this function will first (and only once!) be called like this:
%
% trajectory_generator([],[], 0, path)
%
% i.e. map = varargin{1} and path = varargin{2}.
%
% path: A N x 3 matrix where each row is (x, y, z) coordinate of a
% point in the path. N is the total number of points in the path
%
% This is when you compute and store the trajectory.
%
% Later it will be called with only t and qn as an argument, at
% which point you generate the desired state for point t.
%

desired_state = [];

% use the "persistent" keyword to keep your trajectory around
% inbetween function calls
persistent map waypoints;

if isempty(varargin) == false
    map = varargin{1};
    waypoints = varargin{2}';
else
    % Specify time for each segment
    N = size(waypoints, 2) - 1;
    T_total = 20;
    T = T_total / N;

    t_index = 1;

    for i = 1 : N
        if (t < i * T && t >= (i-1) * T)
            t_index = i;
            break;
        end
    end

    % Compute direction for each segment
    vec = waypoints(:, t_index+1) - waypoints(:, t_index);
    dir = vec / norm(vec);
    
    % =================== Piece-wise 3rd order time polynomials ===================
    A = [1, 0, 0, 0;            % Zero position at beginning
         1, T, T^2, T^3;        % Complete segment at T
         0, 1, 0, 0;            % Zero velocity at beginning
         0, 1, 2*T, 3*T^2];     % Zero velocity at T

    b = [0, norm(vec), 0, 0]';

    alpha = A \ b;

    t_current = t - T * (t_index - 1);
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
            pos = waypoints(:, t_index) + dir * t_polynomial_pos * alpha;
            vel = dir * t_polynomial_vel * alpha;
            acc = dir * t_polynomial_acc * alpha;
            yaw = 0;
            yawdot = 0;
%             pos = [p_x; p_y; p_z];
%             vel = [v_x; v_y; v_z];
%             acc = [a_x; a_y; a_z];
%             yaw = 0;
%             yawdot = 0;
        else
            pos = waypoints(:, end);
            vel = [0; 0; 0];
            acc = [0; 0; 0];
            yaw = 0;
            yawdot = 0;
        end
    end
    % =================== Piece-wise 3rd order time polynomials ===================


%
% When called without varargin (isempty(varargin) == true), compute
% and return the desired state here.
%

desired_state.pos = pos(:);
desired_state.vel = vel(:);
desired_state.acc = acc(:);
desired_state.yaw = yaw;
desired_state.yawdot = yawdot;

end
