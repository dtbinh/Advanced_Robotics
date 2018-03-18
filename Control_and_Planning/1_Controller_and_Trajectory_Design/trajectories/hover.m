function [desired_state] = hover(t, qn)
% HOVER creates hover

% =================== Your code goes here ===================
% You have to set the pos, vel, acc, yaw and yawdot variables
%    
% NOTE: the simulator will spawn the robot to be at the
%       position you return for t == 0

% pos = [0; 2 * t + 0.5 * t * t; 0];
% vel = [0; 2 + t; 0];
% acc = [0; 1; 0];
% yaw = 0;
% yawdot = 0;

if t == 0
    pos = [0; 0; 0];
    vel = [0; 0; 0];
    acc = [0; 0; 0];
    yaw = 0;
    yawdot = 0;
else 
    pos = [0; 0; 10 * t / 1.5];
    vel = [0; 0; 0];
    acc = [0; 0; 0];
    yaw = 0;
    yawdot = 0;
end 

if t > 1.5
    pos = [0; 0; 10];
    vel = [0; 0; 0];
    acc = [0; 0; 0];
    yaw = 0;
    yawdot = 0;
end

% =================== Your code ends here ===================


desired_state.pos = pos(:);
desired_state.vel = vel(:);
desired_state.acc = acc(:);
desired_state.yaw = yaw;
desired_state.yawdot = yawdot;

% if t > 15
%     desired_state.pos = [30; 2 * t - 30; 0];
%     desired_state.vel = [0; 2; 0];
% end
end
