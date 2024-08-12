clear all
close all

% 初始化参数
pursuers_num = 3;
evaders_num = 6;
agents_sum = pursuers_num + evaders_num;
t_step = 0.01;
capture_dis = 0.05;
counter = 0;
L = 1; % 边长为1的正方形

% 创建网格
grid_size = 20;
grid = zeros(grid_size, grid_size);

% 初始化智能体参数
for i = 1:agents_sum
    agents(i).pos = rand(1,2);
    agents(i).active = 1; % evader 存活flag
    agents(i).min_dis = 100;
    agents(i).distance = 0;
end

figure()
while 1
    if sum([agents(pursuers_num+1:agents_sum).active]) == 0
        % 所有的evader都被抓到了
        % 输出视频
        v = VideoWriter('Hunt based on A-star.avi');
        v.FrameRate = 4;
        open(v);
        writeVideo(v, F);
        close(v);
        
        % 计算追捕者的距离总和
        total_distance_pursuers = sum([agents(1:pursuers_num).distance]);
        distances = zeros(1, pursuers_num);
        num_dis = 1;
        
        % 绘制柱状图
        for i = 1:pursuers_num
            distances(num_dis) = agents(i).distance;
            num_dis = num_dis + 1;
        end
        figure;
        
        % 绘制随机颜色
        colors = rand(pursuers_num, 3);
        for i = 1:pursuers_num
            bar(i, distances(i), 'FaceColor', colors(i,:), 'BarWidth', 1.2);
            hold on;
        end
        ylabel('Total Distance Traveled');
        xlabel('Agent Index');
        title('Distance Traveled by Each Agent');
        
        % 显示追捕者的距离总和
        disp(['Total distance traveled by all pursuers: ', num2str(total_distance_pursuers)]);
        return;
    else
        % 重置所有追捕者的min_dis
        for j = 1:pursuers_num
            agents(j).min_dis = 100;
        end

        % 画pursuer和evader的位置
        temp_pos = reshape([agents.pos], 2, agents_sum)';
        plot(temp_pos(1:pursuers_num,1), temp_pos(1:pursuers_num,2), 'go', ...
             temp_pos(pursuers_num+1:end,1), temp_pos(pursuers_num+1:end,2), 'r*');
        
        % 打label
        plabels = arrayfun(@(n) {sprintf('X%d', n)}, (1:agents_sum)');
        hold on
        Hpl = text(temp_pos(:,1), temp_pos(:,2), plabels, ...
              'HorizontalAlignment','left', ...
              'BackgroundColor', 'none');
        xlim([0 1]);
        ylim([0 1]);
       
        % 追捕者移动
        for i = 1:pursuers_num
            if agents(i).active
                % 找到最近的逃避者
                [nearest_evader_pos, ~] = findNearestEvader(agents(i).pos, agents(pursuers_num+1:end));
                
                % 将连续坐标转换为网格坐标
                start = continuousToGrid(agents(i).pos, grid_size);
                goal = continuousToGrid(nearest_evader_pos, grid_size);
                
                % 使用A*算法找到路径
                path = astar(grid, start, goal);
                
                if ~isempty(path) && size(path, 1) > 1
                    % 移动到路径的下一个点
                    next_pos = gridToContinuous(path(2,:), grid_size);
                    old_pos = agents(i).pos;
                    agents(i).pos = next_pos;
                    agents(i).distance = agents(i).distance + norm(next_pos - old_pos);
                end
            end
        end

        % 逃避者移动
        for i = (pursuers_num+1):agents_sum
            if agents(i).active
                % 获取所有追捕者的位置
                pursuer_positions = reshape([agents(1:pursuers_num).pos], 2, [])';
                
                % 计算到所有追捕者的平均方向
                avg_direction = mean(agents(i).pos - pursuer_positions, 1);
                avg_direction = avg_direction / norm(avg_direction);
                
                % 引入随机性
                random_direction = randn(1, 2);
                random_direction = random_direction / norm(random_direction);
                
                % 结合平均方向和随机方向
                escape_direction = 0.8 * avg_direction + 0.2 * random_direction;
                escape_direction = escape_direction / norm(escape_direction);
                
                % 根据最近追捕者的距离调整逃跑距离
                [~, min_distance] = findNearestPursuer(agents(i).pos, agents(1:pursuers_num));
                escape_distance = max(0.1, min(0.3, min_distance / 2));
                
                % 计算目标点，考虑边界
                target = agents(i).pos + escape_direction * escape_distance;
                target = checkBounds(target, L);
                
                % 如果目标点太接近边界，尝试其他方向
                if any(target < 0.1) || any(target > 0.9)
                    alternative_directions = [
                        escape_direction(2), -escape_direction(1);  % 逆时针90度
                        -escape_direction(2), escape_direction(1);  % 顺时针90度
                        -escape_direction(1), -escape_direction(2)  % 180度
                    ];
                    
                    for j = 1:size(alternative_directions, 1)
                        alt_target = agents(i).pos + alternative_directions(j,:) * escape_distance;
                        alt_target = checkBounds(alt_target, L);
                        if all(alt_target >= 0.1) && all(alt_target <= 0.9)
                            target = alt_target;
                            break;
                        end
                    end
                end
                
                % 将连续坐标转换为网格坐标
                start = continuousToGrid(agents(i).pos, grid_size);
                goal = continuousToGrid(target, grid_size);
                
                % 使用A*算法找到路径
                path = astar(grid, start, goal);
                
                if ~isempty(path) && size(path, 1) > 1
                    % 移动到路径的下一个点
                    next_pos = gridToContinuous(path(2,:), grid_size);
                    agents(i).pos = next_pos;
                end
            end
        end

        % 判断是否触发被捕获
        for i = (pursuers_num+1):agents_sum
            if agents(i).active
                agents(i).min_dis = 100; % 重置最小距离
                for j = 1:pursuers_num
                    dis = norm(agents(i).pos - agents(j).pos); 
                    if dis < agents(i).min_dis
                        agents(i).min_dis = dis;
                    end
                end
                % 判断evader被成功捕获的触发条件
                if agents(i).min_dis < capture_dis
                    agents(i).active = 0;
                    hold on
                    plot(agents(i).pos(1,1), agents(i).pos(1,2), 'g*')
                end
            end
        end
        
        hold off
        counter = counter + 1;
        F(counter) = getframe(gcf);
        pause(0.01)
    end
end

% 以下是所有需要的函数定义

function [nearest_pos, nearest_idx] = findNearestEvader(pos, evaders)
    active_evaders = [evaders.active] == 1;
    active_pos = [evaders(active_evaders).pos];
    active_pos = reshape(active_pos, 2, [])';
    distances = vecnorm(active_pos - pos, 2, 2);
    [~, nearest_idx] = min(distances);
    nearest_pos = active_pos(nearest_idx, :);
end

function [nearest_pos, nearest_distance] = findNearestPursuer(pos, pursuers)
    pursuer_pos = [pursuers.pos];
    pursuer_pos = reshape(pursuer_pos, 2, [])';
    distances = vecnorm(pursuer_pos - pos, 2, 2);
    [nearest_distance, nearest_idx] = min(distances);
    nearest_pos = pursuer_pos(nearest_idx, :);
end

function grid_pos = continuousToGrid(pos, grid_size)
    grid_pos = round(pos * (grid_size - 1)) + 1;
    grid_pos = max(min(grid_pos, grid_size), 1);
end

function cont_pos = gridToContinuous(grid_pos, grid_size)
    cont_pos = (grid_pos - 1) / (grid_size - 1);
end

function path = astar(grid, start, goal)
    [rows, cols] = size(grid);
    
    openSet = [start, 0, 0]; % [x, y, g]
    closedSet = zeros(rows, cols);
    cameFrom = zeros(rows, cols, 2);
    
    while ~isempty(openSet)
        [~, current_idx] = min(openSet(:,3) + arrayfun(@(row) heuristic(openSet(row,1:2), goal), 1:size(openSet,1))');
        current = openSet(current_idx, 1:2);
        
        if all(current == goal)
            path = reconstructPath(cameFrom, current);
            return;
        end
        
        current_g = openSet(current_idx, 3);  % 存储当前节点的 g 值
        openSet(current_idx,:) = [];
        closedSet(current(1), current(2)) = 1;
        
        neighbors = getNeighbors(current, rows, cols);
        for i = 1:size(neighbors, 1)
            neighbor = neighbors(i,:);
            
            if closedSet(neighbor(1), neighbor(2))
                continue;
            end
            
            tentative_g = current_g + 1;  % 使用存储的 g 值
            
            neighbor_idx = find(openSet(:,1) == neighbor(1) & openSet(:,2) == neighbor(2));
            if isempty(neighbor_idx)
                openSet = [openSet; neighbor, tentative_g];
            elseif tentative_g < openSet(neighbor_idx,3)
                openSet(neighbor_idx,3) = tentative_g;
            else
                continue;
            end
            
            cameFrom(neighbor(1), neighbor(2),:) = current;
        end
    end
    
    path = [];
end

function h = heuristic(a, b)
    h = norm(a - b);
end

function neighbors = getNeighbors(node, rows, cols)
    directions = [-1 0; 1 0; 0 -1; 0 1];
    neighbors = node + directions;
    valid = neighbors(:,1) > 0 & neighbors(:,1) <= rows & ...
            neighbors(:,2) > 0 & neighbors(:,2) <= cols;
    neighbors = neighbors(valid, :);
end

function path = reconstructPath(cameFrom, current)
    path = current;
    while any(cameFrom(current(1), current(2),:))
        current = squeeze(cameFrom(current(1), current(2),:))';
        path = [current; path];
    end
end

function pos = checkBounds(pos, L)
    if numel(pos) < 2
        error('Position must be a 2-element vector.');
    end
    pos(1) = max(min(pos(1), L), 0); % 检查x坐标
    pos(2) = max(min(pos(2), L), 0); % 检查y坐标
end
