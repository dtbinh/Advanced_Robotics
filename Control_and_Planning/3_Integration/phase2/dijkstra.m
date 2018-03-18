function [path, num_expanded] = dijkstra(map, start, goal, astar)
% DIJKSTRA Find the shortest path from start to goal.
%   PATH = DIJKSTRA(map, start, goal) returns an M-by-3 matrix, where each row
%   consists of the (x, y, z) coordinates of a point on the path.  The first
%   row is start and the last row is goal.  If no path is found, PATH is a
%   0-by-3 matrix.  Consecutive points in PATH should not be farther apart than
%   neighboring cells in the map (e.g.., if 5 consecutive points in PATH are
%   co-linear, don't simplify PATH by removing the 3 intermediate points).
%
%   PATH = DIJKSTRA(map, start, goal, astar) finds the path using euclidean
%   distance to goal as a heuristic if astar is true.
%
%   [PATH, NUM_EXPANDED] = DIJKSTRA(...) returns the path as well as
%   the number of points that were visited while performing the search.

% The default approach is Dijkstra
if nargin < 4
    astar = false;
end

% Initialize the number of visited cells
num_expanded = 0;

% Extract the occupancy grid body
grid = map.occgrid;

% Borders of the indices
border_x = size(grid, 2);
border_y = size(grid, 1);
border_z = size(grid, 3);

% (Nx * Ny * Nz) * 3 array containing the subscripts of all the cells
[I, J, K] = ind2sub(size(grid), 1:border_x * border_y * border_z);

% (Nx * Ny * Nz) * 3 array containing the physical coordinates of each cell
XYZ = map.subToXYZ([I', J', K']);

% Indices of 6 neighbors to consider
Neighbors_6 = zeros(6, border_x * border_y * border_z);
Neighbors_6(1, :) = sub2ind(size(grid), min(border_y, I + 1), J, K);  % south
Neighbors_6(2, :) = sub2ind(size(grid), max(1, I - 1), J, K);         % north
Neighbors_6(3, :) = sub2ind(size(grid), I, min(border_x, J + 1), K);  % right
Neighbors_6(4, :) = sub2ind(size(grid), I, max(1, J - 1), K);         % left
Neighbors_6(5, :) = sub2ind(size(grid), I, J, min(border_z, K + 1));  % up
Neighbors_6(6, :) = sub2ind(size(grid), I, J, max(1, K - 1));         % down

% Indices of 12 neighbors to consider
Neighbors_12 = zeros(12, border_x * border_y * border_z);
Neighbors_12(1, :) = sub2ind(size(grid), min(border_y, I + 1), min(border_x, J + 1), K);  % south-east
Neighbors_12(2, :) = sub2ind(size(grid), min(border_y, I + 1), max(1, J - 1), K);         % south-west
Neighbors_12(3, :) = sub2ind(size(grid), max(1, I - 1), min(border_x, J + 1), K);         % north-east
Neighbors_12(4, :) = sub2ind(size(grid), max(1, I - 1), max(1, J - 1), K);                % north-west

Neighbors_12(5, :) = sub2ind(size(grid), I, min(border_x, J + 1), min(border_z, K + 1));  % up-east
Neighbors_12(6, :) = sub2ind(size(grid), I, min(border_x, J + 1), max(1, K - 1));         % down-east
Neighbors_12(7, :) = sub2ind(size(grid), I, max(1, J - 1), min(border_z, K + 1));         % up-west
Neighbors_12(8, :) = sub2ind(size(grid), I, max(1, J - 1), max(1, K - 1));                % down-west

Neighbors_12(9, :)  = sub2ind(size(grid), min(border_y, I + 1), J, min(border_z, K + 1));  % up-south
Neighbors_12(10, :) = sub2ind(size(grid), min(border_y, I + 1), J, max(1, K - 1));         % down-south
Neighbors_12(11, :) = sub2ind(size(grid), max(1, I - 1), J, min(border_z, K + 1));         % up-north
Neighbors_12(12, :) = sub2ind(size(grid), max(1, I - 1), J, max(1, K - 1));                % down-north

% Indices of 8 neighbors to consider
Neighbors_8 = zeros(8, border_x * border_y * border_z);
Neighbors_8(1, :) = sub2ind(size(grid), min(border_y, I + 1), min(border_x, J + 1), min(border_z, K + 1));
Neighbors_8(2, :) = sub2ind(size(grid), min(border_y, I + 1), min(border_x, J + 1), max(1, K - 1));
Neighbors_8(3, :) = sub2ind(size(grid), min(border_y, I + 1), max(1, J - 1), min(border_z, K + 1));
Neighbors_8(4, :) = sub2ind(size(grid), max(1, I - 1), min(border_x, J + 1), min(border_z, K + 1));

Neighbors_8(5, :) = sub2ind(size(grid), max(1, I - 1), max(1, J - 1), min(border_z, K + 1));
Neighbors_8(6, :) = sub2ind(size(grid), min(border_y, I + 1), max(1, J - 1), max(1, K - 1));
Neighbors_8(7, :) = sub2ind(size(grid), max(1, I - 1), min(border_x, J + 1), max(1, K - 1));
Neighbors_8(8, :) = sub2ind(size(grid), max(1, I - 1), max(1, J - 1), max(1, K - 1));


% Pre-computed collision map and heuristic map for each cell
% collision_map = map.collide(XYZ);
collision_map = grid(:);
heuristic_map = sqrt((XYZ - goal) .* (XYZ - goal));

% Initialize the cell values, records of visited cells and parents of cells
G = Inf(size(grid));
F = Inf(size(grid));
visited = false(size(grid));
parents = zeros(size(grid));

% Indices of start and goal point
ind_start = map.xyzToInd(start);
ind_goal = map.xyzToInd(goal);

G(ind_start) = 0;
F(ind_start) = G(ind_start) + heuristic_map(ind_start);

% Dijkstra planning
if ~astar
    % Get the current closest cell from start
    [min_dist, ind] = min(G(:));
    
    % If there are still possible cells to expand (there exists possible
    % routes from start to goal), continue searching
    while min_dist < Inf
        % Stop if reached the goal
        if ind == ind_goal
            break;
        end

        % Consider 6 neighbors
        for v = 1 : 6
            % index of this neighbor
            ind_checking = Neighbors_6(v, ind);
            % Examine this neighbor and modify the maps
            if (~collision_map(ind_checking)) && (~visited(ind_checking)) && (G(ind_checking) > G(ind) + 1)
                G(ind_checking) = G(ind) + 1;
                parents(ind_checking) = ind;
            end

        end
        
        % Consider 12 neighbors
        for v = 1 : 12
            % index of this neighbor
            ind_checking = Neighbors_12(v, ind);
            % Examine this neighbor and modify the maps
            if (~collision_map(ind_checking)) && (~visited(ind_checking)) && (G(ind_checking) > G(ind) + sqrt(2))
                G(ind_checking) = G(ind) + sqrt(2);
                parents(ind_checking) = ind;
            end

        end
        
        % Consider 8 neighbors
        for v = 1 : 8
            % index of this neighbor
            ind_checking = Neighbors_8(v, ind);
            % Examine this neighbor and modify the maps
            if (~collision_map(ind_checking)) && (~visited(ind_checking)) && (G(ind_checking) > G(ind) + sqrt(3))
                G(ind_checking) = G(ind) + sqrt(3);
                parents(ind_checking) = ind;
            end

        end

        % After examining all 6 neighbors, the visiting of the current cell
        % is completed
        num_expanded = num_expanded + 1;
        % Remove this cell from further consideration
        G(ind) = Inf;
        visited(ind) = true;

        % March to the next cell with minimum distance
        [min_dist, ind] = min(G(:));
    end
    
% A-Star planning
else
    % Get the current closest cell from start
    [min_dist, ind] = min(F(:));
    
    % If there are still possible cells to expand (there exists possible
    % routes from start to goal), continue searching
    while min_dist < Inf
        % Stop if reached the goal
        if ind == ind_goal
            break;
        end
        
        % Consider 6 neighbors
        for v = 1 : 6
            % index of this neighbor
            ind_checking = Neighbors_6(v, ind);
            % Examine this neighbor and modify the maps
            if (~collision_map(ind_checking)) && (~visited(ind_checking)) && (G(ind_checking) > G(ind) + 1)
                G(ind_checking) = G(ind) + 1;
                F(ind_checking) = G(ind_checking) + heuristic_map(ind_checking);
                parents(ind_checking) = ind;
            end

        end
        
        % Consider 12 neighbors
        for v = 1 : 12
            % index of this neighbor
            ind_checking = Neighbors_12(v, ind);
            % Examine this neighbor and modify the maps
            if (~collision_map(ind_checking)) && (~visited(ind_checking)) && (G(ind_checking) > G(ind) + sqrt(2))
                G(ind_checking) = G(ind) + sqrt(2);
                parents(ind_checking) = ind;
            end

        end
        
        % Consider 8 neighbors
        for v = 1 : 8
            % index of this neighbor
            ind_checking = Neighbors_8(v, ind);
            % Examine this neighbor and modify the maps
            if (~collision_map(ind_checking)) && (~visited(ind_checking)) && (G(ind_checking) > G(ind) + sqrt(3))
                G(ind_checking) = G(ind) + sqrt(3);
                parents(ind_checking) = ind;
            end

        end

        % After examining all 6 neighbors, the visiting of the current cell
        % is completed
        num_expanded = num_expanded + 1;
        % Remove this cell from further consideration
        G(ind) = Inf;
        visited(ind) = true;
        % March to the next cell with minimum distance
        [min_dist, ind] = min(G(:));
    end
end

% Initialize the count of waypoints
num_waypoints = 0;
% Start counting the total number of waypoints
last = ind_goal;
while last ~= 0
    last = parents(last);
    num_waypoints = num_waypoints + 1;
end

% Construct the list of waypoints, and fill it up
last = parents(ind_goal);
path = zeros(num_waypoints, 3);
for i = num_waypoints - 1 : -1 : 2
    xyz = map.indToXYZ(last);
    path(i, :) = xyz;
    last = parents(last);
end

% The first and last points in the list are start and goal points
path(1, :) = start;
path(end, :) = goal;

end
