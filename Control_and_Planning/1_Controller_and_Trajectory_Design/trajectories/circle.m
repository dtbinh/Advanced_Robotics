function [desired_state] = circle(t, qn)
% CIRCLE trajectory generator for a circle

% =================== Your code goes here ===================
% You have to set the pos, vel, acc, yaw and yawdot variables
% NOTE: the simulator will spawn the robot to be at the
%       position you return for t == 0
r = 5;
T = 10;

% Constraints are imposed such that:
% dy/dt = 0 when t = 0 and t = T
% d2y/dt2 = 0 when t = 0 and t = T
% theta = 2pi when t = T
% dx/dt = 0, d2x/dt2 = 0 are automatically satisfied when t = 0, T
% The resulting time polynomial is thus of 5th order
% b = [2 * pi; 0; 0; 0; 0];
% A = [T, T ^ 2, T ^ 3, T ^ 4, T ^ 5;
%      1, 2 * T, 3 * T ^ 2, 4 * T ^ 3, 5 * T ^ 4;
%      1, 0, 0, 0, 0;
%      0, 2, 6 * T, 12 * T ^ 2, 20 * T ^ 3;
%      0, 2, 0, 0, 0];
% a = A \ b;
% 
% theta = [t, t ^ 2, t ^ 3, t ^ 4, t ^ 5] * a;
% theta_dot = [1, 2 * t, 3 * t ^ 2, 4 * t ^ 3, 5 * t ^ 4] * a;
% theta_ddot = [0, 2, 6 * T, 12 * T ^ 2, 20 * T ^ 3] * a;


% Constraints are imposed such that:
% dy/dt = 0 when t = 0 and t = T
% theta = 2pi when t = T
% dx/dt = 0, is automatically satisfied when t = 0, T
% The resulting time polynomial is thus of 3rd order
% b = [2 * pi; 0; 0];
% A = [T, T ^ 2, T ^ 3;
%      1, 2 * T, 3 * T ^ 2;
%      1, 0, 0];
% a = A \ b;
% 
% theta = [t, t ^ 2, t ^ 3] * a;
% theta_dot = [1, 2 * t, 3 * t ^ 2] * a;
% theta_ddot = [0, 2, 6 * T] * a;

% Constraints are imposed such that:
% theta = 2pi when t = T
% The resulting time polynomial is thus of 1st order
theta = 2 * pi / T * t;
theta_dot = 2 * pi / T;
theta_ddot = 0;

x = r * cos(theta);
y = r * sin(theta);

dx = - r * sin(theta) * theta_dot;
dy = r * cos(theta) * theta_dot;

ddx = - r * cos(theta) * theta_dot ^ 2 - r * sin(theta) * theta_ddot;
ddy = r * cos(theta) * theta_ddot - r * sin(theta) * theta_dot ^ 2;

z = 2.5 / T * t;

if t == 0
    pos = [r; 0; 0];
    vel = [0; 0; 0];
    acc = [0; 0; 0];
    yaw = 0;
    yawdot = 0;
else
    if t < T
        pos = [x; y; z];
        vel = [dx; dy; 2.5 / T];
        acc = [ddx; ddy; 0];
        yaw = 0;
        yawdot = 0;
    else
        pos = [r; 0; 2.5];
        vel = [0; 0; 0];
        acc = [0; 0; 0];
        yaw = 0;
        yawdot = 0;
    end
end
        

% =================== Your code ends here ===================

desired_state.pos = pos(:);
desired_state.vel = vel(:);
desired_state.acc = acc(:);
desired_state.yaw = yaw;
desired_state.yawdot = yawdot;

end
