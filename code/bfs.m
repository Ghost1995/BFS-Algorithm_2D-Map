function path = bfs(res, start, goal)
% BFS finds the shortest (optimal) path from start to goal.

% PATH = BFS(res, start, goal) returns an M-by-2 matrix, where each row
% consists of the (x, y) coordinates of a point on the path.  The first
% row is start and the last row is goal.  If no path is found, PATH is an
% empty matrix.

% Check if the resolution is correct
if isempty(res)
    disp('No resolution defined');
    path = [];
    return;
end
if (rem(250,res)~=0)||(rem(150,res)~=0)
    disp('The resolution is not correct. Grid cannot be formed.');
    path = [];
    return;
end

% Create a grid to identify obstacles
map = cell(7,1);
map{1} = (res/2):res:250;                                                   % x-values of centre of grid
map{2} = (res/2):res:150;                                                   % y-values of centre of grid
obs1 = [55 112.5; 105 112.5; 105 67.5; 55 67.5];                            % points of interest for obstacle 1
map{3} = [polyfit(obs1(1:2,1),obs1(1:2,2),1);polyfit(obs1(2:3,1),obs1(2:3,2),1);...
          polyfit(obs1(3:4,1),obs1(3:4,2),1);polyfit(obs1([4,1],1),obs1([4,1],2),1)];
obs2 = [120 55; 158 51; 165 89; 188 51; 168 14; 145 14];                    % points of interest for obstacle 2
map{4} = [polyfit(obs2(1:2,1),obs2(1:2,2),1);polyfit(obs2(2:3,1),obs2(2:3,2),1);...
          polyfit(obs2(3:4,1),obs2(3:4,2),1);polyfit(obs2(4:5,1),obs2(4:5,2),1);...
          polyfit(obs2(5:6,1),obs2(5:6,2),1);polyfit(obs2([6,1],1),obs2([6,1],2),1);...
          polyfit(obs2([1,3],1),obs2([1,3],2),1)];
map{5} = [180 120 15];                                                      % points of interest for obstacle 3
map{6} = res;                                                               % resolution
map{7} = false;                                                             % state of points to be tested for collision

% Plot map
figure('units','normalized','outerposition',[0 0 1 1])
axis([0 250 0 150]);
set(gca,'xtick',0:res:250,'ytick',0:res:150)
grid on

hold on
for i=1:length(map{1})
    for j=1:length(map{2})
        % Obstacle 1
        if (map{1}(i)<112.5)&&((map{2}(j)<=map{3}(1,1)*map{1}(i)+map{3}(1,2))&&(map{1}(i)<=(map{2}(j)-map{3}(2,2))/map{3}(2,1))&&...
                               (map{2}(j)>=map{3}(3,1)*map{1}(i)+map{3}(3,2))&&(map{1}(i)>=(map{2}(j)-map{3}(4,2))/map{3}(4,1)))
            rectangle('Position',[map{1}(i)-(res/2) map{2}(j)-(res/2) res res],'FaceColor','k');
        % Obstacle 2
        elseif (map{2}(j)<97.5)&&(((map{1}(i)<(map{2}(j)-map{4}(3,2))/map{4}(3,1))||(map{1}(i)-(res/2)<(map{2}(j)-(res/2)-map{4}(3,2))/map{4}(3,1)))&&...
                                 ((map{1}(i)<(map{2}(j)-map{4}(4,2))/map{4}(4,1))||(map{1}(i)-(res/2)<(map{2}(j)+(res/2)-map{4}(4,2))/map{4}(4,1)))&&...
                                 ((map{2}(j)>map{4}(5,1)*map{1}(i)+map{4}(5,2))||(map{2}(j)+(res/2)>map{4}(5,1)*map{1}(i)+map{4}(5,2)))&&...
                                 ((map{1}(i)>(map{2}(j)-map{4}(6,2))/map{4}(6,1))||(map{1}(i)+(res/2)>(map{2}(j)+(res/2)-map{4}(6,2))/map{4}(6,1)))&&...
                                 ((map{2}(j)<map{4}(7,1)*map{1}(i)+map{4}(7,2))||(map{2}(j)-(res/2)<map{4}(7,1)*map{1}(i)+map{4}(7,2)))&&...
                                 ((map{2}(j)<map{4}(1,1)*map{1}(i)+map{4}(1,2))||(map{2}(j)-(res/2)<map{4}(1,1)*map{1}(i)+map{4}(1,2))||...
                                  (map{1}(i)>(map{2}(j)-map{4}(2,2))/map{4}(2,1))||(map{1}(i)+(res/2)>(map{2}(j)-(res/2)-map{4}(2,2))/map{4}(2,1))))
            rectangle('Position',[map{1}(i)-(res/2) map{2}(j)-(res/2) res res],'FaceColor','k');
        % Obstacle 3
        elseif (((map{1}(i)-map{5}(1))^2 + (map{2}(j)-map{5}(2))^2)<(map{5}(3)^2))||...
               (((map{1}(i)+(res/2)-map{5}(1))^2 + (map{2}(j)+(res/2)-map{5}(2))^2)<(map{5}(3)^2))||...
               (((map{1}(i)+(res/2)-map{5}(1))^2 + (map{2}(j)-(res/2)-map{5}(2))^2)<(map{5}(3)^2))||...
               (((map{1}(i)-(res/2)-map{5}(1))^2 + (map{2}(j)-(res/2)-map{5}(2))^2)<(map{5}(3)^2))||...
               (((map{1}(i)-(res/2)-map{5}(1))^2 + (map{2}(j)+(res/2)-map{5}(2))^2)<(map{5}(3)^2))
            rectangle('Position',[map{1}(i)-(res/2) map{2}(j)-(res/2) res res],'FaceColor','k');
        end
    end
end
% Real Obstacle 1
plot(obs1(1:2,1),obs1(1:2,2),'w')
plot(obs1(2:3,1),obs1(2:3,2),'w')
plot(obs1(3:4,1),obs1(3:4,2),'w')
plot(obs1([4,1],1),obs1([4,1],2),'w')
% Real Obstacle 2
plot(obs2(1:2,1),obs2(1:2,2),'w')
plot(obs2(2:3,1),obs2(2:3,2),'w')
plot(obs2(3:4,1),obs2(3:4,2),'w')
plot(obs2(4:5,1),obs2(4:5,2),'w')
plot(obs2(5:6,1),obs2(5:6,2),'w')
plot(obs2([6,1],1),obs2([6,1],2),'w')
% Real Obstacle 3
t = linspace(0,2*pi,1000);
plot(map{5}(1)+map{5}(3)*cos(t),map{5}(2)+map{5}(3)*sin(t),'w')

% If start is empty, ask the user to set start point
if isempty(start)
    title('No start point was mentioned. Select a start point in the grid.');
    start = ginput(1);
    title('');
end
% Convert the given start point to grid coordinates
[C_start,start_pos] = collide(map,start);
if C_start == 1
    uiwait(msgbox('The start point does not lie in the free space of the grid.'));
    close all
    path = [];
    return;
end
rectangle('Position',[start_pos(1)-(res/2) start_pos(2)-(res/2) res res],'FaceColor','r');

% If goal is empty, ask the user to set goal point
if isempty(goal)
    title('No goal point was mentioned. Select a goal point in the grid.');
    goal = ginput(1);
    title('');
end
% Convert the given goal point to grid coordinates
[C_goal,goal_pos] = collide(map,goal);
if C_goal == 1
    uiwait(msgbox('The goal point does not lie in the free space of the grid.'));
    close all
    path = [];
    return;
end
rectangle('Position',[goal_pos(1)-(res/2) goal_pos(2)-(res/2) res res],'FaceColor','b');

% Set the points to be in grid coords
map{7} = true;
% Set the initial node
init_node.pos = start_pos;
init_node.parent = struct;
% Start an OPEN list
OPEN = init_node;
% Define all possible actions
if start_pos(1)<=goal_pos(1)
    if start_pos(2)<=goal_pos(2)
        actions = [res, 0; res, res; 0, res; -res, res; -res, 0; -res, -res; 0, -res; res, -res];
    else
        actions = [res, 0; res, -res; 0, -res; -res, -res; -res, 0; -res, res; 0, res; res, res];
    end
else
    if start_pos(2)<=goal_pos(2)
        actions = [-res, 0; -res, res; 0, res; res, res; res, 0; res, -res; 0, -res; -res, -res];
    else
        actions = [-res, 0; -res, -res; 0, -res; res, -res; res, 0; res, res; 0, res; -res, res];
    end
end
% Start a CLOSED list
CLOSED = zeros(length(map{1}),length(map{2}));
% Start a path list
path = goal_pos;
tol = res/1e4;
% Search algorithm
% index = 0;
while ~isempty(OPEN)
%     index = index+1;
    current_node = OPEN(1);
    OPEN(1) = [];
    
    if all(abs(current_node.pos - goal_pos)<tol)
        parent_node = current_node.parent;
        while any(abs(parent_node.pos - start_pos)>tol)
            path = [path; parent_node.pos];
            parent_node = parent_node.parent;
        end
        break;
    end
    
    x = find(abs(map{1}-current_node.pos(1))<tol,1);
    y = find(abs(map{2}-current_node.pos(2))<tol,1);
    if CLOSED(x,y) == 1
        continue;
    end
    CLOSED(x,y) = 1;
    % expand
    for i=1:length(actions)
        neighbor_node.pos = current_node.pos + actions(i,:);
        % check if it intersects with an obstacle (also checks if it lies
        % inside the boundary of the map)
        if collide(map,neighbor_node.pos)
            continue;
        end
        % check if closed already
        if CLOSED(abs(map{1}-neighbor_node.pos(1))<tol,abs(map{2}-neighbor_node.pos(2))<tol)
            continue;
        end
        neighbor_node.parent = current_node;
        if any(abs(neighbor_node.pos - goal_pos)>tol)
            rectangle('Position',[neighbor_node.pos(1)-(res/2) neighbor_node.pos(2)-(res/2) res res],'FaceColor','y');
        end
        pause(0.001)
        OPEN = [OPEN; neighbor_node];
    end
end
path = [path; start_pos];
path = flipud(path);
map{7} = false;
% Recheck path for collisions
if any(collide(map,path))
    uiwait(msgbox('The path found is not correct.'));
    close all
    path = [];
    return;
end
% Plot path
title('Path Found')
for i=2:(size(path,1)-1)
    rectangle('Position',[path(i,1)-(res/2) path(i,2)-(res/2) res res],'FaceColor','c');
    pause(0.01)
end
plot(path(:,1),path(:,2))
hold off

end