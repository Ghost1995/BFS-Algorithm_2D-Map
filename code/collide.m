function [C,varargout] = collide(map, points)
% COLLIDE tests whether points collide with an obstacle in an environment.

% C = collide(map, points), where points is an M-by-2 matrix with each row
% as an (x, y) point. C is an M-by-1 logical vector.
% C(i) = 1 if M(i, :) touches an obstacle and is 0 otherwise.

% If the map is empty, output empty vector
if isempty(map)
    C = [];
else
    C = zeros(size(points,1),1);
    if map{7}
        tol = map{6}/1e4;
        for i=1:length(C)
            x_grid = find(abs(map{1}-points(i,1))<tol,1);
            if isempty(x_grid)
                C(i,1) = 1;
                continue;
            end
            y_grid = find(abs(map{2}-points(i,2))<tol,1);
            if isempty(y_grid)
                C(i,1) = 1;
                continue;
            end
        end
    else
        varargout{1} = [];
        for i=1:length(C)
            x_grid = find(points(i,1)<=map{1},1);
            if isempty(x_grid)
                if points(i,1) > map{1}(end)+(map{6}/2)
                    C(i,1) = 1;
                    continue;
                else
                    x_grid = length(map{1});
                end
            elseif points(i,1)+(map{6}/2) < map{1}(x_grid)
                if x_grid~=1
                    x_grid = x_grid - 1;
                else
                    C(i,1) = 1;
                    continue;
                end
            end
            y_grid = find(points(i,2)<=map{2},1);
            if isempty(y_grid)
                if points(i,2) > map{2}(end)+(map{6}/2)
                    C(i,1) = 1;
                    continue;
                else
                    y_grid = length(map{2});
                end
            elseif points(i,2)+(map{6}/2) < map{2}(y_grid)
                if y_grid~=1
                    y_grid = y_grid - 1;
                else
                    C(i,1) = 1;
                    continue;
                end
            end
            points(i,:) = [map{1}(x_grid), map{2}(y_grid)];
        end
        varargout{1} = points;
    end
    for i=1:length(C)
        if C(i)~=1
            if points(i,1)<112.5
                % Obstacle 1
                if (points(i,2)<=map{3}(1,1)*points(i,1)+map{3}(1,2))&&(points(i,1)<=(points(i,2)-map{3}(2,2))/map{3}(2,1))&&...
                   (points(i,2)>=map{3}(3,1)*points(i,1)+map{3}(3,2))&&(points(i,1)>=(points(i,2)-map{3}(4,2))/map{3}(4,1))
                    C(i,1) = 1;
                    continue;
                end
            elseif points(i,2)<97.5
                % Obstacle 2
                if ((points(i,1)<(points(i,2)-map{4}(3,2))/map{4}(3,1))||(points(i,1)-(map{6}/2)<(points(i,2)-(map{6}/2)-map{4}(3,2))/map{4}(3,1)))&&...
                   ((points(i,1)<(points(i,2)-map{4}(4,2))/map{4}(4,1))||(points(i,1)-(map{6}/2)<(points(i,2)+(map{6}/2)-map{4}(4,2))/map{4}(4,1)))&&...
                   ((points(i,2)>map{4}(5,1)*points(i,1)+map{4}(5,2))||(points(i,2)+(map{6}/2)>map{4}(5,1)*points(i,1)+map{4}(5,2)))&&...
                   ((points(i,1)>(points(i,2)-map{4}(6,2))/map{4}(6,1))||(points(i,1)+(map{6}/2)>(points(i,2)+(map{6}/2)-map{4}(6,2))/map{4}(6,1)))&&...
                   ((points(i,2)<map{4}(7,1)*points(i,1)+map{4}(7,2))||(points(i,2)-(map{6}/2)<map{4}(7,1)*points(i,1)+map{4}(7,2)))&&...
                   ((points(i,2)<map{4}(1,1)*points(i,1)+map{4}(1,2))||(points(i,2)-(map{6}/2)<map{4}(1,1)*points(i,1)+map{4}(1,2))||...
                    (points(i,1)>(points(i,2)-map{4}(2,2))/map{4}(2,1))||(points(i,1)+(map{6}/2)>(points(i,2)-(map{6}/2)-map{4}(2,2))/map{4}(2,1)))
                    C(i,1) = 1;
                    continue;
                end
            else
                % Obstacle 3
                if (((points(i,1)-map{5}(1))^2 + (points(i,2)-map{5}(2))^2)<(map{5}(3)^2))||...
                   (((points(i,1)+(map{6}/2)-map{5}(1))^2 + (points(i,2)+(map{6}/2)-map{5}(2))^2)<(map{5}(3)^2))||...
                   (((points(i,1)+(map{6}/2)-map{5}(1))^2 + (points(i,2)-(map{6}/2)-map{5}(2))^2)<(map{5}(3)^2))||...
                   (((points(i,1)-(map{6}/2)-map{5}(1))^2 + (points(i,2)-(map{6}/2)-map{5}(2))^2)<(map{5}(3)^2))||...
                   (((points(i,1)-(map{6}/2)-map{5}(1))^2 + (points(i,2)+(map{6}/2)-map{5}(2))^2)<(map{5}(3)^2))
                    C(i,1) = 1;
                    continue;
                end
            end
        end
    end
end

end