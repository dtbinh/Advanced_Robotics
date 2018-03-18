function [desired_state] = diamond(t, qn)
% DIAMOND trajectory generator for a diamond

% =================== Your code goes here ===================
% You have to set the pos, vel, acc, yaw and yawdot variables
% NOTE: the simulator will spawn the robot to be at the
%       position you return for t == 0

% Specify waypoints
waypoints = [0, 0, 0;
             1/4, sqrt(2), sqrt(2);
             2/4, 0, 2 * sqrt(2);
             3/4, -sqrt(2), sqrt(2);
             1, 0, 0]';

% Specify time for each segment
N = size(waypoints, 2) - 1;
segment_dirs = diff(waypoints, 1, 2);
segment_lengths = sqrt(sum(segment_dirs .* segment_dirs));
total_length = sum(segment_lengths);

T_total = 8;
T_segments = T_total / total_length * segment_lengths;

t_index = 1;

for i = 1 : N
    if (t < sum(T_segments(1:i)) && t >= sum(T_segments(1:i-1)))
        t_index = i;
        break;
    end
end

% Compute direction for each segment
vec = waypoints(:, t_index+1) - waypoints(:, t_index);
dir = vec / norm(vec);

% =================== Minimum Acceleration Trajectory (3rd order splines) ===================
% A = zeros(4 * N, 4 * N);
% b = zeros(4 * N, 3);
% 
% for i = 1 : N
%     A(i, :) = [zeros(1, 4*(i-1)), 1, zeros(1, 4*(N-i)+3)];
%     b(i, :) = waypoints(:, i);
%     
%     A(i + N, :) = [zeros(1, 4*(i-1)), ones(1, 4), zeros(1, 4*(N-i))];
%     b(i + N, :) = waypoints(:, i + 1);
% end
% 
% for i = 1 : N - 1
%     A(i + 2*N, :) = [zeros(1, 4*(i-1)), 0, 1/T_segments(i), 2/T_segments(i), 3/T_segments(i), 0, -1/T_segments(i + 1), zeros(1, 4*(N-i)-2)];
%     A(i + 3*N - 1, :) = [zeros(1, 4*(i-1)), 0, 0, 2/T_segments(i)^2, 6/T_segments(i)^2, 0, 0, -2/T_segments(i + 1)^2, zeros(1, 4*(N-i)-3)];
% end
% 
% A(4*N - 1, 2) = 1 / T_segments(1);
% A(4*N, 4*N-2:4*N) = [1/T_segments(N), 2/T_segments(N), 3/T_segments(N)];
% 
% alpha = A \ b;
% alpha_x = alpha(:, 1);
% alpha_y = alpha(:, 2);
% alpha_z = alpha(:, 3);
% 
% 
% alpha_x_current = alpha_x(1 + 4*(t_index - 1) : 4 + 4*(t_index - 1));
% alpha_y_current = alpha_y(1 + 4*(t_index - 1) : 4 + 4*(t_index - 1));
% alpha_z_current = alpha_z(1 + 4*(t_index - 1) : 4 + 4*(t_index - 1));
% 
% t_current = t - sum(T_segments(1: t_index-1));
% T = T_segments(t_index);
% t_polynomial_pos = [1, t_current/T, (t_current/T) ^ 2, (t_current/T) ^ 3];
% t_polynomial_vel = [0, 1/T, 2 * (t_current/T) / T, 3 * (t_current/T) ^ 2 / T];
% t_polynomial_acc = [0, 0, 2 / T ^ 2, 6 * (t_current/T) / T];
% 
% p_x = t_polynomial_pos * alpha_x_current;
% p_y = t_polynomial_pos * alpha_y_current;
% p_z = t_polynomial_pos * alpha_z_current;
% 
% v_x = t_polynomial_vel * alpha_x_current;
% v_y = t_polynomial_vel * alpha_y_current;
% v_z = t_polynomial_vel * alpha_z_current;
% 
% a_x = t_polynomial_acc * alpha_x_current;
% a_y = t_polynomial_acc * alpha_y_current;
% a_z = t_polynomial_acc * alpha_z_current;
% =================== Minimum Acceleration Trajectory (3rd order splines) ===================



% =================== Piece-wise 5th order time polynomials ===================
% A = [1, 0, 0, 0, 0, 0;                    % Zero position at beginning
%      1, T, T^2, T^3, T^4, T^5;            % Complete segment at T
%      0, 1, 0, 0, 0, 0;                    % Zero velocity at beginning
%      0, 1, 2*T, 3*T^2, 4*T^3, 5*T^4;      % Zero velocity at T
%      0, 0, 2, 0, 0, 0;                    % Zero acceleration at beginning
%      0, 0, 2, 6*T, 12*T^2, 20*T^3];       % Zero acceleration at T
% 
% b = [0, norm(vec), 0, 0, 0, 0]';
% 
% alpha = A \ b;
% 
% t_current = t - T * (t_index - 1);
% t_polynomial_pos = [1, t_current, t_current ^ 2, t_current ^ 3, t_current ^ 4, t_current ^ 5];
% t_polynomial_vel = [0, 1, 2 * t_current, 3 * t_current ^ 2, 4 * t_current ^ 3, 5 * t_current ^ 4];
% t_polynomial_acc = [0, 0, 2, 6 * t_current, 12 * t_current ^ 2, 20 * t_current * 3];
% =================== Piece-wise 5th order time polynomials ===================

% =================== Piece-wise 3rd order time polynomials ===================
T = T_segments(t_index);
A = [1, 0, 0, 0;            % Zero position at beginning
     1, T, T^2, T^3;        % Complete segment at T
     0, 1, 0, 0;            % Zero velocity at beginning
     0, 1, 2*T, 3*T^2];     % Zero velocity at T

b = [0, norm(vec), 0, 0]';

alpha = A \ b;

t_current = t - sum(T_segments(1:t_index-1));
t_polynomial_pos = [1, t_current, t_current ^ 2, t_current ^ 3];
t_polynomial_vel = [0, 1, 2 * t_current, 3 * t_current ^ 2];
t_polynomial_acc = [0, 0, 2, 6 * t_current];

if t == 0
    pos = [0; 0; 0];
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
%         pos = [p_x; p_y; p_z];
%         vel = [v_x; v_y; v_z];
%         acc = [a_x; a_y; a_z];
%         yaw = 0;
%         yawdot = 0;
    else
        pos = [1; 0; 0];
        vel = [0; 0; 0];
        acc = [0; 0; 0];
        yaw = 0;
        yawdot = 0;
    end
end
% =================== Piece-wise 3rd order time polynomials ===================


% =================== Your code ends here ===================

desired_state.pos = pos(:);
desired_state.vel = vel(:);
desired_state.acc = acc(:);
desired_state.yaw = yaw;
desired_state.yawdot = yawdot;

end

