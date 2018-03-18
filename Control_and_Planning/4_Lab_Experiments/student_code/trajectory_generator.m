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
pos=[];
vel=[];
acc=[];
yaw=0;
yawdot=0;
end_t=20;

% use the "persistent" keyword to keep your trajectory around
% inbetween function calls
persistent c t_list path; 
% Variables which will be used after the first call:
% 1. matrix recording constant parameters of cubic spline
% 2. list of time passing through path points
% 3. path

if ~isempty(varargin)
    path=varargin{2};
    N=size(path, 1); % number of points
    
    % calculate the time at each corner based on the arc length of path
    distance=0;
    t_list=zeros(N,1);
    t_list(1)=0;
    for i=2:N
        distance=distance+norm(path(i,:)-path(i-1,:));
        t_list(i)=end_t*distance;
    end
    t_list=t_list/distance;
    
    c=zeros(N-1, 4, 3);
    
    % use cubic spline to generate trajectory, solve for 4(N-1) DoFs
    % matrix A should remain the same regardless of coordinate
    A=zeros(4*(N-1));
    % merge the start point and end point constraints in first 4 rows
    A(1, :)=[0, 0, 0, 1, zeros(1, 4*(N-2))];
    A(2, :)=[0, 0, 1, 0, zeros(1, 4*(N-2))];
    A(3, :)=[zeros(1, 4*(N-2)), t_list(N)^3, t_list(N)^2, t_list(N), 1];
    A(4, :)=[zeros(1, 4*(N-2)), 3*t_list(N)^2, 2*t_list(N), 1, 0];
    % left 4(N-2) constraints
    for j=2:N-1
        A(4*j-3, :)=[zeros(1, 4*(j-2)), t_list(j)^3, t_list(j)^2, t_list(j), 1, zeros(1, 4*(N-j))];
        A(4*j-2, :)=[zeros(1, 4*(j-1)), t_list(j)^3, t_list(j)^2, t_list(j), 1, zeros(1, 4*(N-j-1))];
        A(4*j-1, :)=[zeros(1, 4*(j-2)), 3*t_list(j)^2, 2*t_list(j), 1, 0, -3*t_list(j)^2, -2*t_list(j), -1, 0, zeros(1, 4*(N-j-1))];
        A(4*j, :)=[zeros(1, 4*(j-2)), 6*t_list(j), 2, 0, 0, -6*t_list(j), -2, 0, 0, zeros(1, 4*(N-j-1))];
    end
    
    % vector b depends on which coordinate we solve
    for i=1:3  % solve the constants for x, y, z coordinates respectively
        b=zeros(4*(N-1), 1);
        % merge the start point and end point constraints in first 4 rows
        b(1)=path(1, i);
        %b(2)=0;
        b(3)=path(N, i);
        %b(4)=0;
        % left 4(N-2) constraints
        for j=2:N-1
            b(4*j-3, :)=path(j, i);
            b(4*j-2, :)=path(j, i);
            %b(4*j-1, :)=0;
            %b(4*j, :)=0;
        end
        c_unarranged=A\b;
        c_arranged=zeros(4, N-1);
        c_arranged(:)=c_unarranged(:);
        c(:,:,i)=c_arranged';
    end
%
% When called without varargin (isempty(varargin) == true), compute
% and return the desired state here.
%
else
    pos=zeros(3, 1);
    vel=zeros(3, 1);
    acc=zeros(3, 1);
    if t<0
        t=0;
    end
    for i=1:length(t_list)
        if i==length(t_list)
            pos=path(end,:)';
        elseif t>=t_list(i) && t<=t_list(i+1)
            for j=1:3
                pos(j)=c(i, :, j)*[t^3 t^2 t 1]';
                vel(j)=c(i, :, j)*[3*t^2 2*t 1 0]';
                acc(j)=c(i, :, j)*[6*t 2 0 0]';
            end
            break;
        end
    end

end

desired_state.pos = pos(:);
desired_state.vel = vel(:);
desired_state.acc = acc(:);
desired_state.yaw = yaw;
desired_state.yawdot = yawdot;

end
