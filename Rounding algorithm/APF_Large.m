clear all
close all
health = 100; % 初始血量
health_depletion_rate = 8; % 每秒损失的血量
pursuers_num = 5;
evaders_num = 1;
agents_sum = pursuers_num + evaders_num;
t_step = 0.01;
capture_dis = 0.05;
counter = 0;
L = 10; % 边长为L的正方形
square_x = [0 0 L L 0]; % 5个顶点的坐标,第一个和最后一个重合才能形成封闭图形
square_y = [0 L L 0 0];
% 最大速度
max_speed = 2.5; % 设置最大速度
% 初始化智能体参数
for i = 1:agents_sum
    agents(i).pos = rand(1,2)*L;
    agents(i).active = 1; % evader 存活flag
    agents(i).min_dis = 100;
    agents(i).distance = 0;
end

% 势场参数
k_att = 3; % 吸引力系数
k_rep = 0.5; % 排斥力系数
rep_range = 0.015; % 排斥力作用范围
k_wall = 0.5; % 墙壁斥力系数
wall_range = 0.1; % 墙壁斥力作用范围

% 新参数
cooperation_range = 0.3; % 协作范围
prediction_factor = 1; % 预测因子
noise_amplitude = 2; % 添加噪声幅度
inertia = 0.9; % 添加惯性因子

% MPC 参数
prediction_horizon = 10;  % 预测时域
control_horizon = 5;      % 控制时域
Q = diag([10, 10]);       % 状态权重矩阵
R = 0.1;                  % 控制输入权重

% 虚拟质点参数
num_virtual_points = pursuers_num;  % 虚拟质点数量等于追捕者数量
virtual_point_radius = 0.75;  % 虚拟质点分布半径

figure()
while 1
    if sum([agents(pursuers_num+1:agents_sum).active]) == 0 % 所有的evader都被抓到了
        % 输出视频
        v = VideoWriter('Hunt based on APF.avi');
        v.FrameRate = 4;
        open(v);
        writeVideo(v, F);
        close(v);

        % 计算追捕者的距离总和
        total_distance_pursuers = sum([agents(1:pursuers_num).distance]);
        
        % 收集追捕者的距离数据
        distances = zeros(1, pursuers_num);
        for i = 1:pursuers_num
            distances(i) = agents(i).distance;
        end
        % 计算追捕者的距离总和
        num_dis=1;
        % 绘制柱状图
        for i=1:agents_sum
            if agents(i).active==1
                distances(num_dis)=agents(i).distance;
                num_dis=num_dis+1;
            end
        end
        figure;

        %绘制随机颜色,生成一个 pursuers_num x 3 的矩阵，其中每行代表一个随机颜色（RGB值）。
        colors = rand(pursuers_num, 3);
        for i=1:pursuers_num
            bar(distances(i),colors(i,:),BarWidth=1.2);
        end
        ylabel('Total Distance Traveled');
        xlabel('Agent Index');
        title('Distance Traveled by Each Agent');
        % 显示追捕者的距离总和
        disp(['Total distance traveled by all pursuers: ', num2str(total_distance_pursuers)]);
        return;
    else
        % 画pursuer和evader的位置
        temp_pos = reshape([agents.pos], 2, agents_sum)';
        
        % 更新虚拟质点位置
        evader_pos = mean([agents(pursuers_num+1:agents_sum).pos], 1);
        virtual_points = updateVirtualPoints(evader_pos, num_virtual_points, virtual_point_radius);
        
        % 计算追捕者到虚拟质点的距离矩阵
        cost_matrix = zeros(pursuers_num, num_virtual_points);
        for i = 1:pursuers_num
            for j = 1:num_virtual_points
                cost_matrix(i, j) = norm(agents(i).pos - virtual_points(j, :));
            end
        end

        % 使用匈牙利算法进行分配
        assignments = munkres(cost_matrix);
        
        plot(temp_pos(1:pursuers_num,1), temp_pos(1:pursuers_num,2), 'go', ...
             temp_pos(pursuers_num+1:end,1), temp_pos(pursuers_num+1:end,2), 'r*', ...
             virtual_points(:,1), virtual_points(:,2), 'b.');
        
        % 打label
        plabels = arrayfun(@(n) {sprintf('X%d', n)}, (1:agents_sum)');
        hold on
        Hpl = text(temp_pos(:,1), temp_pos(:,2), plabels, ...
              'HorizontalAlignment','left', ...
              'BackgroundColor', 'none');
        xlim([0 L]);
        ylim([0 L]);

        % 追捕者的 MPC 控制
        max_speed_pursuer = 1.2; % 略微增加最大速度
        for i = 1:pursuers_num
            assigned_point_index = find(assignments(i, :));
            if ~isempty(assigned_point_index)
                target_point = virtual_points(assigned_point_index, :);
            else
                % 如果没有分配到虚拟质点，直接追踪最近的逃避者
                evader_positions = [agents(pursuers_num+1:end).pos];
                distances = vecnorm(agents(i).pos - evader_positions, 2, 2);
                [~, nearest_evader] = min(distances);
                target_point = agents(pursuers_num + nearest_evader).pos;
            end
            
            % 计算人工势场力
            F_rep = [0, 0];
            for j = 1:pursuers_num
                if i ~= j
                    diff = calculateDistance(agents(i).pos, agents(j).pos, L);
                    dist = norm(diff);
                    if dist < rep_range
                        F_rep = F_rep + k_rep * (1/dist - 1/rep_range) * (1/dist^2) * (diff / dist);
                    end
                end
            end
            
            % 结合MPC和APF
            [optimal_control, predicted_trajectory] = mpcControlWithAPF(agents(i).pos, target_point, F_rep, prediction_horizon, control_horizon, Q, R, max_speed_pursuer);
            
            % 保存旧位置
            old_pos = agents(i).pos;
            
            % 更新追捕者位置
            agents(i).pos = agents(i).pos + t_step * optimal_control(1:2);
            agents(i).pos = checkBounds(agents(i).pos, L);
            
            % 更新移动距离
            agents(i).distance = agents(i).distance + norm(agents(i).pos - old_pos);
            
            % 绘制追捕者到目标点的连接线
            line([agents(i).pos(1), target_point(1)], [agents(i).pos(2), target_point(2)], 'Color', 'k', 'LineStyle', ':');
        end

        % 在主循环外部初始化逃逸者的目标点和当前方向
        for i = (pursuers_num+1):agents_sum
            agents(i).target = rand(1, 2) * L;
            agents(i).direction = (agents(i).target - agents(i).pos) / norm(agents(i).target - agents(i).pos);
        end

        for i = (pursuers_num+1):agents_sum
            if agents(i).active
                % 计算到目标点的距离
                dist_to_target = norm(agents(i).target - agents(i).pos);
                
                % 如果接近目标点，选择新的目标点
                if dist_to_target < 0.5
                    agents(i).target = rand(1, 2) * L;
                    agents(i).direction = (agents(i).target - agents(i).pos) / norm(agents(i).target - agents(i).pos);
                end
                
                % 计算墙壁斥力
                F_wall = calculateWallRepulsion(agents(i).pos, L, k_wall, wall_range);
                
                % 更新方向，考虑当前方向、目标方向和墙壁斥力
                target_direction = (agents(i).target - agents(i).pos) / norm(agents(i).target - agents(i).pos);
                new_direction = 0.7 * agents(i).direction + 0.2 * target_direction + 0.1 * F_wall;
                new_direction = new_direction / norm(new_direction);
                
                % 平滑过渡到新方向
                agents(i).direction = 2 * agents(i).direction + 0.8 * new_direction;
                agents(i).direction = agents(i).direction / norm(agents(i).direction);
                
                % 更新速度和位置
                agents(i).velocity = agents(i).direction * max_speed;
                new_pos = agents(i).pos + t_step * agents(i).velocity;
                agents(i).pos = checkBounds(new_pos, L);
                
                % 如果位置被边界限制，调整方向
                if any(agents(i).pos ~= new_pos)
                    agents(i).direction = agents(i).direction - 2 * dot(agents(i).direction, F_wall) * F_wall;
                    agents(i).direction = agents(i).direction / norm(agents(i).direction);
                end
            end
        end
        
        % 检查虚拟质点是否被占领并扣血
        occupied_points = 0;
        for j = 1:num_virtual_points
            for i = 1:pursuers_num
                if norm(agents(i).pos - virtual_points(j, :)) < capture_dis
                    occupied_points = occupied_points + 1;
                    break;
                end
            end
        end

        % 扣血
        health = health - occupied_points * health_depletion_rate * t_step;
        health = max(0, health); % 确保血量不会变成负数
        
        % 显示血条
        health_bar_length = 50; % 血条长度
        health_percentage = health / 100;
        health_bar = ['[' repmat('=', 1, round(health_percentage * health_bar_length)) ...
                      repmat(' ', 1, health_bar_length - round(health_percentage * health_bar_length)) ']'];
        health_text = sprintf('Health: %s %.1f%%', health_bar, health);
        title(health_text);
        
        % 检查是否满足被围捕条件
        if health <= 0
            disp('完成本次作战目标');
            % 在这里可以添加游戏结束的处理，比如停止模拟、显示结果等
            break; % 退出主循环
        end
        
        hold off
        counter = counter + 1;
        F(counter) = getframe(gcf);
        pause(0.01)
    end
end

% 边界检查和调整函数
function pos = checkBounds(pos, L)
    pos(1) = max(0, min(pos(1), L));
    pos(2) = max(0, min(pos(2), L));
end

% 计算两点之间的最短距离（考虑边界）
function diff = calculateDistance(pos1, pos2, L)
    diff = pos1 - pos2;
    diff = diff - L * round(diff / L);
end

% 计算墙壁斥力
function F_wall = calculateWallRepulsion(pos, L, k_wall, wall_range)
    F_wall = [0, 0];
    
    % 左墙
    if pos(1) < wall_range
        F_wall(1) = F_wall(1) + k_wall * (1/pos(1) - 1/wall_range) * (1/pos(1)^2);
    end
    
    % 右墙
    if L - pos(1) < wall_range
        F_wall(1) = F_wall(1) - k_wall * (1/(L-pos(1)) - 1/wall_range) * (1/(L-pos(1))^2);
    end
    
    % 下墙
    if pos(2) < wall_range
        F_wall(2) = F_wall(2) + k_wall * (1/pos(2) - 1/wall_range) * (1/pos(2)^2);
    end
    
    % 上墙
    if L - pos(2) < wall_range
        F_wall(2) = F_wall(2) - k_wall * (1/(L-pos(2)) - 1/wall_range) * (1/(L-pos(2))^2);
    end
end

function virtual_points = updateVirtualPoints(center, num_points, radius)
    angles = linspace(0, 2*pi, num_points+1);
    angles = angles(1:end-1);  % 去掉最后一个重复的点
    x = cos(angles) * radius + center(1);
    y = sin(angles) * radius + center(2);
    virtual_points = [x; y]';
end

% MPC 控制函数，结合APF
function [optimal_control, predicted_trajectory] = mpcControlWithAPF(current_pos, target_pos, F_rep, prediction_horizon, control_horizon, Q, R, max_speed)
    predicted_trajectory = zeros(prediction_horizon, 2);
    predicted_trajectory(1, :) = current_pos;
    
    for i = 2:prediction_horizon
        if i <= control_horizon + 1
            direction = target_pos - predicted_trajectory(i-1, :);
            if norm(direction) > 0
                direction = direction / norm(direction);
            else
                direction = [0, 0];
            end
            
            % 结合APF力
            combined_direction = direction + F_rep;
            if norm(combined_direction) > 0
                combined_direction = combined_direction / norm(combined_direction);
            end
            
            control = combined_direction * max_speed;
        else
            control = [0, 0];
        end
        predicted_trajectory(i, :) = predicted_trajectory(i-1, :) + control;
    end
    
    % 计算最优控制
    direction = target_pos - current_pos;
    if norm(direction) > 0
        optimal_direction = direction / norm(direction);
    else
        optimal_direction = [0, 0];
    end
    
    % 结合APF力
    combined_direction = optimal_direction + F_rep;
    if norm(combined_direction) > 0
        combined_direction = combined_direction / norm(combined_direction);
    end
    
    optimal_control = combined_direction * max_speed;
end


function assignment = munkres(costMat)
    [m, n] = size(costMat);
    assignment = zeros(m, n);
    costMat(costMat == inf) = 1e+9;
    
    % Step 1: Subtract row minima
    costMat = costMat - min(costMat, [], 2);
    
    % Step 2: Subtract column minima
    costMat = costMat - min(costMat, [], 1);
    
    % Step 3: Cover all zeros with a minimum number of lines
    coveredRows = false(m, 1);
    coveredCols = false(1, n);
    starZ = false(m, n);
    primeZ = false(m, n);
    
    for i = 1:m
        for j = 1:n
            if costMat(i,j) == 0 && ~coveredRows(i) && ~coveredCols(j)
                starZ(i,j) = true;
                coveredRows(i) = true;
                coveredCols(j) = true;
            end
        end
    end
    
    while sum(coveredRows) + sum(coveredCols) < m
        % Step 4: Find an uncovered zero
        zeroRow = 0;
        zeroCol = 0;
        for i = 1:m
            for j = 1:n
                if costMat(i,j) == 0 && ~coveredRows(i) && ~coveredCols(j)
                    zeroRow = i;
                    zeroCol = j;
                    break;
                end
            end
            if zeroRow ~= 0
                break;
            end
        end
        
        if zeroRow == 0
            % Step 5: Find the smallest uncovered value
            minVal = min(min(costMat(~coveredRows, ~coveredCols)));
            costMat(~coveredRows, :) = costMat(~coveredRows, :) - minVal;
            costMat(:, coveredCols) = costMat(:, coveredCols) + minVal;
        else
            primeZ(zeroRow, zeroCol) = true;
            starCol = find(starZ(zeroRow, :), 1);
            if isempty(starCol)
                % Step 6: Augment path
                while true
                    starZ(zeroRow, zeroCol) = true;
                    primeZ(zeroRow, zeroCol) = false;
                    starRow = find(starZ(:, zeroCol), 1);
                    if isempty(starRow)
                        break;
                    end
                    zeroRow = starRow;
                    zeroCol = find(primeZ(zeroRow, :), 1);
                end
                coveredRows(:) = false;
                coveredCols(:) = false;
                starZ(primeZ) = true;
                primeZ(:) = false;
            else
                coveredRows(zeroRow) = true;
                coveredCols(starCol) = false;
            end
        end
    end
    
    assignment = starZ;
end
