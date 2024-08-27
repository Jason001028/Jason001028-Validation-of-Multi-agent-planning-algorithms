clear all
close all

pursuers_num = 3;
evaders_num = 1;
agents_sum = pursuers_num + evaders_num;
t_step = 0.01;
capture_dis = 0.05;
counter = 0;
L = 9; % 边长为3的正方形
square_x = [0 0 L L 0]; % 5个顶点的坐标,第一个和最后一个重合才能形成封闭图形
square_y = [0 L L 0 0];
% 最大速度
max_speed = 0.25; % 设置最大速度
% 初始化智能体参数
for i = 1:agents_sum
    agents(i).pos = rand(1,2) * L; % 修改初始位置范围
    agents(i).active = 1; % evader 存活flag
    agents(i).min_dis = 100;
    agents(i).distance = 0;
end

% 势场参数
k_att = 3 % 吸引力系数
k_rep = 0.5; % 排斥力系数
rep_range = 0.015; % 排斥力作用范围
k_wall = 0.5; % 墙壁斥力系数
wall_range = 0.1; % 墙壁斥力作用范围

% 新参数
cooperation_range = 0.3 ; % 协作范围
prediction_factor = 1; % 预测因子
noise_amplitude = 2; % 添加噪声幅度
inertia = 0.9; % 添加惯性因子
vision_range = 1.5; % 视野范围

% 初始化迷雾
fog_resolution = 0.1;
fog_grid = zeros(ceil(L/fog_resolution), ceil(L/fog_resolution));  % 初始化为全黑（0）

% 在主循环开始之前，为每个围捕者清除初始视野范围内的迷雾
for i = 1:pursuers_num
    fog_grid = clearFog(agents(i).pos, vision_range, fog_grid, fog_resolution, L);
end

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
        % 清除当前视野内的迷雾（仅对围捕者）
        for i = 1:pursuers_num
            if agents(i).active
                fog_grid = clearFog(agents(i).pos, vision_range, fog_grid, fog_resolution, L);
            end
        end
        
        % 绘制地图和迷雾
        imagesc([0 L], [0 L], fog_grid');
        colormap([0 0 0; 1 1 1]);  % 黑色为未探索区域，白色为已探索区域
        hold on;
        
        % 画pursuer和evader的位置
        temp_pos = reshape([agents.pos], 2, agents_sum)';
        plot(temp_pos(1:pursuers_num,1), temp_pos(1:pursuers_num,2), 'go', ...
             temp_pos(pursuers_num+1:end,1), temp_pos(pursuers_num+1:end,2), 'r*');
        
        % 画视野范围
        for i = 1:agents_sum
            if agents(i).active
                viscircles(agents(i).pos, vision_range, 'Color', 'b', 'LineStyle', ':');
            end
        end
        
        % 打label
        plabels = arrayfun(@(n) {sprintf('X%d', n)}, (1:agents_sum)');
        Hpl = text(temp_pos(:,1), temp_pos(:,2), plabels, ...
              'HorizontalAlignment','left', ...
              'BackgroundColor', 'none');
        xlim([0 L]);
        ylim([0 L]);
       
        % 初始化 last_direction
        if ~isfield(agents, 'last_direction')
            for i = 1:pursuers_num
                agents(i).last_direction = [0, 0];
            end
        end

        % 追捕者的力计算
        for i = 1:pursuers_num
            F_att = [0, 0];
            F_rep = [0, 0];
            F_coop = [0, 0];
            F_wall = [0, 0];
            nearest_evader_dist = inf;
            nearest_evader_pos = [0, 0];
            nearest_evader_vel = [0, 0];
            
            % 目标选择：找到视野范围内最近的未被捕获且未被其他追捕者盯上的逃避者
            for j = (pursuers_num+1):agents_sum
                if agents(j).active
                    diff = calculateDistance(agents(j).pos, agents(i).pos, L);
                    dist = norm(diff);
                    if dist <= vision_range  % 只考虑视野范围内的逃避者
                        is_targeted = false;
                        for k = 1:pursuers_num
                            if k ~= i
                                diff_k = calculateDistance(agents(k).pos, agents(j).pos, L);
                                if norm(diff_k) < dist
                                    is_targeted = true;
                                    break;
                                end
                            end
                        end
                        if ~is_targeted && dist < nearest_evader_dist
                            nearest_evader_dist = dist;
                            nearest_evader_pos = agents(j).pos;
                            if isfield(agents(j), 'velocity')
                                nearest_evader_vel = agents(j).velocity;
                            else
                                nearest_evader_vel = [0, 0];
                            end
                        end
                    end
                end
            end
            
            % 计算吸引力（包括预测）
            if nearest_evader_dist < inf  % 只有当找到有效目标时才计算吸引力
                predicted_pos = nearest_evader_pos + prediction_factor * nearest_evader_vel;
                diff = calculateDistance(predicted_pos, agents(i).pos, L);
                dist = norm(diff);
                k_att_adjusted = k_att * (1 + 1 / dist); % 距离越近，吸引力越大
                F_att = k_att_adjusted * diff / dist;
            end
            
            % 计算排斥力（避免碰撞）
            for j = 1:agents_sum
                if i ~= j
                    diff = calculateDistance(agents(j).pos, agents(i).pos, L);
                    dist = norm(diff);
                    if dist < rep_range
                        F_rep = F_rep - k_rep * (1/dist - 1/rep_range) * (1/dist^2) * diff / dist;
                    end
                end
            end
            
            % 计算协作力
            for j = 1:pursuers_num
                if i ~= j
                    diff = calculateDistance(agents(j).pos, agents(i).pos, L);
                    dist = norm(diff);
                    if dist < cooperation_range
                        F_coop = F_coop + 0.5 * diff / dist; % 轻微吸引，形成包围之势
                    end
                end
            end
            
            % 计算墙壁斥力
            F_wall = calculateWallRepulsion(agents(i).pos, L, k_wall, wall_range);
            
            % 合力，要保证协作力足够小，不能大于吸引力
            F_total = F_att + F_rep + 0.2*F_coop + F_wall;
            
            % 添加随机噪声（减小幅度）
            noise = 0.5 * noise_amplitude * (rand(1,2) - 0.5);
            F_total = F_total + noise;
            
            % 添加惯性（减小影响）
            F_total = F_total + 0.3 * inertia * agents(i).last_direction;
            
            % 归一化力
            if norm(F_total) > 0
                F_total = F_total / norm(F_total);
            end
            
            agents(i).up = F_total;
            agents(i).last_direction = F_total;
        end

        % 更新追捕者位置
        max_speed_pursuer = 0.8; % 略微增加最大速度
        acceleration = 0.15; % 加速度
        for i = 1:pursuers_num
            if agents(i).active
                if ~isfield(agents(i), 'velocity') || isempty(agents(i).velocity)
                    agents(i).velocity = [0, 0];
                end

                % 确保 up 是行向量
                agents(i).up = agents(i).up(:)';
                
                % 保存旧位置
                old_pos = agents(i).pos;
                
                % 更新速度
                agents(i).velocity = agents(i).velocity + acceleration * agents(i).up;
                
                % 限制最大速度
                speed = norm(agents(i).velocity);
                if speed > max_speed_pursuer
                    agents(i).velocity = max_speed_pursuer * agents(i).velocity / speed;
                end
                
                % 更新位置
                new_pos = agents(i).pos + t_step * agents(i).velocity;
                agents(i).pos = checkBounds(new_pos, L);
                
                % 更新移动距离
                agents(i).distance = agents(i).distance + norm(agents(i).pos - old_pos);
            end
        end

        % 更新逃避者位置
        for i = (pursuers_num+1):agents_sum
            if agents(i).active
                % 确保 velocity 存在并且是正确的形状
                if ~isfield(agents(i), 'velocity') || isempty(agents(i).velocity)
                    agents(i).velocity = [0, 0];
                elseif numel(agents(i).velocity) ~= 2
                    warning('Agent %d velocity has unexpected size. Resetting to [0, 0].', i);
                    agents(i).velocity = [0, 0];
                else
                    agents(i).velocity = agents(i).velocity(:)'; % 确保是行向量
                end
                
                % 计算逃避方向（只考虑视野范围内的追捕者）
                escape_direction = [0, 0];
                for j = 1:pursuers_num
                    diff = calculateDistance(agents(i).pos, agents(j).pos, L);
                    dist = norm(diff);
                    if dist <= vision_range
                        escape_direction = escape_direction + diff / (dist^2);
                    end
                end
                
                % 添加墙壁斥力
                F_wall = calculateWallRepulsion(agents(i).pos, L, k_wall, wall_range);
                escape_direction = escape_direction + F_wall;
                
                % 确保 escape_direction 是非零向量
                if norm(escape_direction) > 0
                    escape_direction = escape_direction / norm(escape_direction);
                else
                    escape_direction = [rand(), rand()]; % 如果为零，随机选择一个方向
                    escape_direction = escape_direction / norm(escape_direction);
                end
                
                % 更新速度
                agents(i).velocity = 0.7 * agents(i).velocity + 0.3 * escape_direction;
                
                % 限制最大速度
                speed = norm(agents(i).velocity);
                if speed > max_speed
                    agents(i).velocity = max_speed * agents(i).velocity / speed;
                end
                
                % 更新位置
                new_pos = agents(i).pos + t_step * agents(i).velocity;
                agents(i).pos = checkBounds(new_pos, L);

                % 如果位置被边界限制，调整速度
                if any(agents(i).pos ~= new_pos)
                    agents(i).velocity = (agents(i).pos - old_pos) / t_step;
                end
                
                % 添加小幅随机扰动以避免局部极大值
                if rand() < 0.1 % 10%的概率添加扰动
                    agents(i).pos = agents(i).pos + 0.01 * (rand(1,2) - 0.5);
                    agents(i).pos = checkBounds(agents(i).pos, L);
                end
            end
        end
 
        % 判断是否触发被捕获
        for i = (pursuers_num+1):agents_sum
            if agents(i).active
                agents(i).min_dis = 100; % 重置最小距离
                for j = 1:pursuers_num
                    diff = calculateDistance(agents(i).pos, agents(j).pos, L);
                    dis = norm(diff);
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

% 边界检查和调整函数
function pos = checkBounds(pos, L)
    pos(1) = max(0, min(pos(1), L));
    pos(2) = max(0, min(pos(2), L));
end

% 修改 clearFog 函数
function fog_grid = clearFog(pos, vision_range, fog_grid, fog_resolution, L)
    [rows, cols] = size(fog_grid);
    for i = 1:rows
        for j = 1:cols
            cell_pos = [(j-1)*fog_resolution, (i-1)*fog_resolution];
            dist = norm(calculateDistance(cell_pos, pos, L));
            if dist <= vision_range
                fog_grid(i, j) = 0;  % 0 表示已探索（黑色）
            end
        end
    end
end

function diff = calculateDistance(pos1, pos2, L)
    diff = pos1 - pos2;
    diff = diff - L * round(diff / L);  % 周期性边界条件
end

function F_wall = calculateWallRepulsion(pos, L, k_wall, wall_range)
    F_wall = [0, 0];
    
    % 检查并计算四面墙的排斥力
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

