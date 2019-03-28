%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This code generates the optimal path using A* algorithm for a point robot
% 
% Input:
%           res --> Resolution for the grid
%         start --> Start position
%          goal --> Goal position
%   createVideo --> States whether the video has to be created
% 
% Output:
%   path --> Optimal path from given start to a given goal
% 
% Submitted by: Ashwin Goyal (UID - 115526297)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function path = astar_proj2(res, start, goal, createVideo)

    % Check if the resolution is correct
    if isempty(res)
        disp('No resolution defined');
        path = [];
        return;
    elseif (rem(250,res)~=0)||(rem(150,res)~=0)
        disp('The resolution is not correct. Grid cannot be formed.');
        path = [];
        return;
    end
    
    % Create a grid to identify obstacles
    map = cell(7,1);
    map{1} = (res/2):res:250;                                               % x-values of centre of grid
    map{2} = (res/2):res:150;                                               % y-values of centre of grid
    obs1 = [55 112.5; 105 112.5; 105 67.5; 55 67.5];                        % points of interest for obstacle 1
    map{3} = [polyfit(obs1(1:2,1),obs1(1:2,2),1);polyfit(obs1(2:3,1),obs1(2:3,2),1);...
              polyfit(obs1(3:4,1),obs1(3:4,2),1);polyfit(obs1([4,1],1),obs1([4,1],2),1)];
    obs2 = [120 55; 158 51; 165 89; 188 51; 168 14; 145 14];                % points of interest for obstacle 2
    map{4} = [polyfit(obs2(1:2,1),obs2(1:2,2),1);polyfit(obs2(2:3,1),obs2(2:3,2),1);...
              polyfit(obs2(3:4,1),obs2(3:4,2),1);polyfit(obs2(4:5,1),obs2(4:5,2),1);...
              polyfit(obs2(5:6,1),obs2(5:6,2),1);polyfit(obs2([6,1],1),obs2([6,1],2),1);...
              polyfit(obs2([1,3],1),obs2([1,3],2),1)];
    map{5} = [180 120 15];                                                  % points of interest for obstacle 3
    map{6} = res;                                                           % resolution
    map{7} = false;                                                         % state of points to be tested for collision
    
    % Plot map
    figure('units','normalized','outerposition',[0 0 1 1])
    axis([0 250 0 150]);
    if res > min([map{1}(end) map{2}(end)])*0.02
        set(gca,'xtick',0:res:250,'ytick',0:res:150)
        grid on
        
        hold on
        for i=1:length(map{1})
            for j=1:length(map{2})
                % Obstacle 1
                if (map{1}(i)<112.5)&&((map{2}(j)<=map{3}(1,1)*map{1}(i)+map{3}(1,2))&&...
                                       (map{1}(i)<=(map{2}(j)-map{3}(2,2))/map{3}(2,1))&&...
                                       (map{2}(j)>=map{3}(3,1)*map{1}(i)+map{3}(3,2))&&...
                                       (map{1}(i)>=(map{2}(j)-map{3}(4,2))/map{3}(4,1)))
                    rectangle('Position',[map{1}(i)-(res/2) map{2}(j)-(res/2) res res],'FaceColor','k');
                % Obstacle 2
                elseif (map{2}(j)<97.5)&&(((map{1}(i)<(map{2}(j)-map{4}(3,2))/map{4}(3,1))||...
                                           (map{1}(i)-(res/2)<(map{2}(j)-(res/2)-map{4}(3,2))/map{4}(3,1)))&&...
                                          ((map{1}(i)<(map{2}(j)-map{4}(4,2))/map{4}(4,1))||...
                                           (map{1}(i)-(res/2)<(map{2}(j)+(res/2)-map{4}(4,2))/map{4}(4,1)))&&...
                                          ((map{2}(j)>map{4}(5,1)*map{1}(i)+map{4}(5,2))||...
                                           (map{2}(j)+(res/2)>map{4}(5,1)*map{1}(i)+map{4}(5,2)))&&...
                                          ((map{1}(i)>(map{2}(j)-map{4}(6,2))/map{4}(6,1))||...
                                           (map{1}(i)+(res/2)>(map{2}(j)+(res/2)-map{4}(6,2))/map{4}(6,1)))&&...
                                          ((map{2}(j)<map{4}(7,1)*map{1}(i)+map{4}(7,2))||...
                                           (map{2}(j)-(res/2)<map{4}(7,1)*map{1}(i)+map{4}(7,2)))&&...
                                          ((map{2}(j)<map{4}(1,1)*map{1}(i)+map{4}(1,2))||...
                                           (map{2}(j)-(res/2)<map{4}(1,1)*map{1}(i)+map{4}(1,2))||...
                                           (map{1}(i)>(map{2}(j)-map{4}(2,2))/map{4}(2,1))||...
                                           (map{1}(i)+(res/2)>(map{2}(j)-(res/2)-map{4}(2,2))/map{4}(2,1))))
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
    else
        hold on
        % Real Obstacle 1
        patch(obs1(:,1),obs1(:,2),'k')
        % Real Obstacle 2
        patch(obs2(:,1),obs2(:,2),'k')
        % Real Obstacle 3
        t = linspace(0,2*pi,1000);
        patch(map{5}(1)+map{5}(3)*cos(t),map{5}(2)+map{5}(3)*sin(t),'k')
    end
    
    % If start is empty, ask the user to set start point
    if isempty(start)
        title('No start point was mentioned. Select a start point in the grid.');
        start = ginput(1);
        title('');
    end
    % Convert the given start point to grid coordinates
    [C_start,start_pos] = collide_proj2(map,start);
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
    [C_goal,goal_pos] = collide_proj2(map,goal);
    if C_goal == 1
        uiwait(msgbox('The goal point does not lie in the free space of the grid.'));
        close all
        path = [];
        return;
    end
    rectangle('Position',[goal_pos(1)-(res/2) goal_pos(2)-(res/2) res res],'FaceColor','b');
    
    % Create a video
    if createVideo
        vidObj = VideoWriter(['../output/astar_proj2 demo_run (res = ' num2str(res) ').mp4'],'MPEG-4');
        vidObj.FrameRate = 30;
        open(vidObj);
        writeVideo(vidObj, getframe(gcf));
    end
    
    % Set the points to be in grid coords
    map{7} = true;
    title('Running A* Algorithm')
    pause(0.1)
    
    % Set the initial node
    init_node.pos = start_pos;
    init_node.gcost = 0;
    init_node.hcost = norm(start_pos-goal_pos);
    init_node.fcost = init_node.gcost + init_node.hcost;
    init_node.parent = struct;
    
    % Start an OPEN list
    OPEN = containers.Map(init_node.fcost,init_node);
    k = init_node.fcost;
    
    % Define all possible actions
    actions = [res, 0; res, res; 0, res; -res, res; -res, 0; -res, -res; 0, -res; res, -res];
    
    % Start a CLOSED list
    CLOSED = zeros(length(map{1}),length(map{2}));
    
    % Start a path list
    path = goal_pos;
    tol = res/1e4;
    
    % Search algorithm
    while ~isempty(OPEN)
        if length(OPEN(min(k)))~=1
            OPEN_temp = OPEN(min(k));
            current_node = OPEN_temp(1);
            OPEN_temp(1:end-1) = OPEN_temp(2:end);
            OPEN_temp(end) = [];
            OPEN(min(k)) = OPEN_temp;
        else
            current_node = OPEN(min(k));
            remove(OPEN,min(k));
            k = k(abs(k-min(k))>tol);
        end
        
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
            % check if it intersects with an obstacle (also checks if it
            % lies inside the boundary of the map)
            if collide_proj2(map,neighbor_node.pos)
                continue;
            end
            % check if closed already
            if CLOSED(abs(map{1}-neighbor_node.pos(1))<tol,abs(map{2}-neighbor_node.pos(2))<tol)
                continue;
            end
            neighbor_node.gcost = current_node.gcost + norm(neighbor_node.pos-current_node.pos);
            neighbor_node.hcost = norm(neighbor_node.pos - goal_pos);
            neighbor_node.fcost = neighbor_node.gcost + neighbor_node.hcost;
            neighbor_node.parent = current_node;
            if isKey(OPEN,neighbor_node.fcost)
                OPEN(neighbor_node.fcost) = [OPEN(neighbor_node.fcost);neighbor_node];
            else
                OPEN(neighbor_node.fcost) = neighbor_node;
                k(end+1) = neighbor_node.fcost;
            end
            if any(abs(neighbor_node.pos - goal_pos)>tol)
                rectangle('Position',[neighbor_node.pos(1)-(res/2) neighbor_node.pos(2)-(res/2) res res],'FaceColor','y');
                if createVideo
                    writeVideo(vidObj, getframe(gcf));
                end
                pause(0.001)
            end
        end
    end
    path = [path; start_pos];
    path = flipud(path);
    
    % Plot path
    title('Path Found')
    for i=2:(size(path,1)-1)
        rectangle('Position',[path(i,1)-(res/2) path(i,2)-(res/2) res res],'FaceColor','c');
        if createVideo
            writeVideo(vidObj, getframe(gcf));
        end
        pause(0.01)
    end
    plot(path(:,1),path(:,2))
    hold off
    
    if createVideo
        for i = 1:5*vidObj.FrameRate
            writeVideo(vidObj, getframe(gcf));
        end
        close(vidObj);
    end
    
    % Save the figure
    saveas(gcf,['../output/astar_path_proj2 (res = ' num2str(res) ').jpg']);

end