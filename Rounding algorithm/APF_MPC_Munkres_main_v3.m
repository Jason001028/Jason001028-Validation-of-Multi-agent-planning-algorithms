% 主程序开始
clear all
close all

% 初始化参数
health = 100; % 初始血量
health_depletion_rate = 8; % 每秒损失的血量
pursuers_num = 5;
evaders_num = 1;
agents_sum = pursuers_num + evaders_num;
t_step = 0.01;
%虚拟质点被占领距离阈值
capture_dis = 0.25;
counter = 0;
L = 10; % 边长为L的正方形

% MPC 参数
prediction_horizon = 10;
control_horizon = 5;
Q = diag([2, 1]); % 状态权重矩阵
R = diag([0.1, 0.1]); % 控制权重矩阵
max_speed = 2.8; % 最大速度

% 人工势场参数
rep_range = 2;
k_rep = 1;
%障碍物斥力生效范围
obs_range = 0.75;
k_obs = 2;

vehicles = cell(1, pursuers_num);
for i = 1:pursuers_num
    position = rand(1,2)*L;
    velocity = [0, 0];
    acceleration = [0, 0];
    orientation = rand(1) * 2 * pi;
    length = 0.2; % 车辆长度，根据需要调整
    width = 0.1; % 车辆宽度，根据需要调整
    vehicles{i} = Vehicle(position, velocity, acceleration, orientation, length, width);
end

% 逃逸目标参数
evader_speed = 0.35; % 逃逸目标的低速运动速度
num_evader_virtual_points = pursuers_num; % 逃逸目标周围的虚拟质点数量
evader_virtual_point_radius = 1.5; % 逃逸目标虚拟质点的分布半径
evader_virtual_point_speed = 0.75; % 逃逸目标虚拟质点的旋转速度
esa_flag = 0;%代表是逃逸者
dy_flag = 1;%代表是动态障碍物
% 初始化动态障碍物
%动态障碍物数量,参考逃避者
n_dy = 2;
evader_speed_dy = 4  % 动态障碍物的中高速运动速度
for i = 1:n_dy
    obstacles_dy(i).pos = rand(1,2)*L/2 + L/4;
    obstacles_dy(i).vel = [0 0];
    obstacles_dy(i).acc = [0 0];
    obstacles_dy(i).direction = rand(1,2);
    obstacles_dy(i).direction = obstacles_dy(i).direction / norm(obstacles_dy(i).direction); % 归一化方向向量
end


% 初始化逃避者
for i = pursuers_num+1:agents_sum
    agents(i).pos = rand(1,2)*L/2 + L/4;
    agents(i).vel = [0 0];
    agents(i).acc = [0 0];
    agents(i).health = 100;
    agents(i).active = 1;
    agents(i).direction = rand(1,2);
    agents(i).direction = agents(i).direction / norm(agents(i).direction); % 归一化方向向量
end

% 初始化障碍物
%障碍物数量
n = 10;
obstacles = rand(n,2)*L;

% 初始化计数器和帧
counter = 0;
F = {};%改为cell数组

figure()
while 1
    if sum([agents(pursuers_num+1:agents_sum).active]) == 0
        disp('Mission Acomplished!')
        break
    else

        clf; % 清除当前图形
        hold on; % 开始绘图

        % 更新逃逸目标位置
        evader_pos = agents(pursuers_num+1).pos;
        evader_direction = agents(pursuers_num+1).direction;
        new_evader_pos = evader_pos + evader_speed * evader_direction * t_step;
        agents(pursuers_num+1).pos = checkBounds(new_evader_pos, L);


        % 更新逃逸目标周围的虚拟质点
        evader_virtual_points = updateEvaderVirtualPoints(evader_pos, num_evader_virtual_points, evader_virtual_point_radius, evader_virtual_point_speed, counter * t_step);

        % 更新和绘制 vehicles
        for i = 1:pursuers_num
            % 绘制 vehicle 为矩形
            drawVehicle(vehicles{i});
        end
        
        % 绘制逃逸目标和其虚拟质点
        plot(evader_pos(1), evader_pos(2), 'r*', 'MarkerSize', 10);
        hold on;
        plot(evader_virtual_points(:,1), evader_virtual_points(:,2), 'ro', 'MarkerSize', 5);
        
        % 绘制障碍物
        plot(obstacles(:,1),obstacles(:,2),'ks','MarkerSize',10,'MarkerFaceColor','k')
        
        % 更新动态障碍物
        for i = 1:n_dy
                % 计算动态障碍物的逃跑方向
                pursuer_positions = cell2mat(cellfun(@(v) v.position, vehicles, 'UniformOutput', false));
                pursuer_positions = reshape(pursuer_positions, [], 2);  % 确保是 Nx2 的矩阵
                escape_direction = calculateEscapeDirection(obstacles_dy(i).pos, pursuer_positions, obstacles,L,dy_flag);
        
                % 更新动态障碍物的方向、速度和位置
                obstacles_dy(i).direction = escape_direction;
                obstacles_dy(i).vel = obstacles_dy(i).direction * evader_speed_dy;
                obstacles_dy(i).pos = obstacles_dy(i).pos + obstacles_dy(i).vel * t_step;
                obstacles_dy(i).pos = checkBounds(obstacles_dy(i).pos, L);
                % 绘制动态障碍物
                plot(obstacles_dy(i).pos(1),obstacles_dy(i).pos(2),'o','MarkerSize',10,'MarkerFaceColor',[0.7 0.7 0.7],'MarkerEdgeColor',[0.5 0.5 0.5])
        end
        

        % 设置图形属性
        axis([0 L 0 L])
        title(['Time: ',num2str(counter*t_step,'%.2f'),'s'])
        
        % 更新虚拟质点位置
        evader_pos = mean([agents(pursuers_num+1:agents_sum).pos], 1);
        
        % 计算 vehicles 到虚拟质点的距离矩阵
        % 使用 evader_virtual_points 替代 virtual_points
        cost_matrix = zeros(pursuers_num, num_evader_virtual_points);
        for i = 1:pursuers_num
            for j = 1:num_evader_virtual_points
                cost_matrix(i, j) = norm(vehicles{i}.position - evader_virtual_points(j, :));
            end
        end

        % 使用匈牙利算法进行分配
        assignments = munkres(cost_matrix);

        % 更新逃避者
        for i = pursuers_num+1:agents_sum
            if agents(i).active
                % 计算逃避者的逃跑方向
                pursuer_positions = cell2mat(cellfun(@(v) v.position, vehicles, 'UniformOutput', false));
                pursuer_positions = reshape(pursuer_positions, [], 2);  % 确保是 Nx2 的矩阵
                escape_direction = calculateEscapeDirection(agents(i).pos, pursuer_positions, obstacles,L,esa_flag);
        
                % 更新逃避者的方向、速度和位置
                agents(i).direction = escape_direction;
                agents(i).vel = agents(i).direction * evader_speed;
                agents(i).pos = agents(i).pos + agents(i).vel * t_step;
                agents(i).pos = checkBounds(agents(i).pos, L);
                
                % 检查虚拟质点是否被占领并扣血
                occupied_points = 0;
                for j = 1:num_evader_virtual_points
                    for k = 1:pursuers_num
                        if norm(vehicles{k}.position - evader_virtual_points(j, :)) < capture_dis
                            occupied_points = occupied_points + 1;
                            break;
                        end
                    end
                end
                % 扣血
                agents(i).health = agents(i).health - occupied_points * health_depletion_rate * t_step;
                agents(i).health = max(0, agents(i).health); % 确保血量不会变成负数
                
                % 显示血条
                health_bar_length = 50;
                health_percentage = agents(i).health / 100;
                health_bar = ['[' repmat('=', 1, round(health_percentage * health_bar_length)) ...
                              repmat(' ', 1, health_bar_length - round(health_percentage * health_bar_length)) ']'];
                health_text = sprintf('Health: %s %.1f%%', health_bar, agents(i).health);
                title(health_text);
        
                % 检查是否被捕获
                if agents(i).health <= 0
                    agents(i).active = 0;
                    disp(['Evader ', num2str(i-pursuers_num), ' captured!']);
                end
            end
        end

        % Vehicles的决策规划 
        for i = 1:pursuers_num
            assigned_point_index = find(assignments(i, :));
            if ~isempty(assigned_point_index)
                target_point = evader_virtual_points(assigned_point_index, :);
            else
                % 如果没有分配到虚拟质点,选择最近的虚拟质点
                [~, nearest_point] = min(vecnorm(vehicles{i}.position - evader_virtual_points, 2, 2));
                target_point = evader_virtual_points(nearest_point, :);
            end
            
            % 计算人工势场力
            F_rep = calculateRepulsiveForce(vehicles, i, obstacles, n, rep_range, k_rep, obs_range, k_obs,n_dy,obstacles_dy);
            
            % 结合MPC和APF
            [optimal_control, predicted_trajectory] = ModulePredictControlWithAPF(vehicles{i}, target_point, F_rep, max_speed, L,evader_pos,evader_virtual_point_speed);
        
            % 更新 vehicle 状态
            vehicles{i} = updateVehicle(vehicles{i}, optimal_control, t_step, L);
            
            % 检查碰撞
            checkVehicleCollisions(vehicles, i, obstacles);
        end
        
        
        % 绘制 vehicles 到分配的虚拟质点的虚线
        for i = 1:pursuers_num
            assigned_point_index = find(assignments(i, :));
            if ~isempty(assigned_point_index)
                target_point = evader_virtual_points(assigned_point_index, :);
                % 绘制虚线
                plot([vehicles{i}.position(1), target_point(1)], [vehicles{i}.position(2), target_point(2)], 'b--');
            end
        end

        % 计算被占领的虚拟质点数目
        occupied_points = 0;
        for j = 1:num_evader_virtual_points
            for k = 1:pursuers_num
                if norm(vehicles{k}.position - evader_virtual_points(j, :)) < capture_dis
                    occupied_points = occupied_points + 1;
                    break;
                end
            end
        end

        % 在右上角显示被占领的虚拟质点数目
        text_pos = [L*0.25, L*0.05]; % 文本位置
        text_str = sprintf('Occupied: %d/%d', occupied_points, num_evader_virtual_points);
        text(text_pos(1), text_pos(2), text_str, 'FontSize', 12, 'FontWeight', 'bold', 'HorizontalAlignment', 'right');
        
        hold off
        counter = counter + 1;
        F{counter} = getframe(gcf);  % 使用 cell 数组存储帧
        pause(0.01)
    end
end

% 辅助函数定义开始

function drawVehicle(vehicle)
    % 计算车辆的四个角点
    corners = calculateVehicleCorners(vehicle);
    
    % 绘制车辆主体
    fill(corners(:,1), corners(:,2), 'b');
    
    % 绘制方向指示线
    direction_line = [vehicle.position; vehicle.position + [cos(vehicle.orientation), sin(vehicle.orientation)] * vehicle.length];
    plot(direction_line(:,1), direction_line(:,2), 'r', 'LineWidth', 2);
    
    hold on;
end

function corners = calculateVehicleCorners(vehicle)
    % 计算车辆的四个角点
    R = [cos(vehicle.orientation), -sin(vehicle.orientation);
         sin(vehicle.orientation), cos(vehicle.orientation)];
    corners = [-vehicle.length/2, -vehicle.width/2;
               vehicle.length/2, -vehicle.width/2;
               vehicle.length/2, vehicle.width/2;
               -vehicle.length/2, vehicle.width/2];
    corners = (corners * R) + repmat(vehicle.position, 4, 1);
end

function F_rep = calculateRepulsiveForce(vehicles, current_index, obstacles, n, rep_range, k_rep, obs_range, k_obs,n_dy,obstacles_dy)
    F_rep = [0, 0];
    current_pos = vehicles{current_index}.position;
    
    % 计算来自其他车辆的斥力
    for i = 1:length(vehicles)
        if i ~= current_index
            d = norm(current_pos - vehicles{i}.position);
            if d < rep_range
                F_rep = F_rep + k_rep * (1/d - 1/rep_range) * (1/d^2) * (current_pos - vehicles{i}.position) / d;
            end
        end
    end
    
    % 计算来自障碍物的斥力
    for i = 1:n
        d = norm(current_pos - obstacles(i,:));
        if d < obs_range
            F_rep = F_rep + k_obs * (1/d - 1/obs_range) * (1/d^2) * (current_pos - obstacles(i,:)) / d;
        end
    end
    
    % 计算来自动态障碍物的斥力
    for i = 1:n_dy
        x=obstacles_dy(i).pos(1);
        y=obstacles_dy(i).pos(2);
        pos_dy=[x,y];
        d = norm(current_pos - pos_dy);
        if d < obs_range
            F_rep = F_rep + k_obs * (1/d - 1/obs_range) * (1/d^2) * (current_pos - pos_dy) / d;
        end
    end

end

%%%%%%%%%%%%%%%%%%%%%%%%%                 MPC+APF,关键代码                 %%%%%%%%%%%%%%%%%%%%%%%%%%%
function [optimal_control, predicted_trajectory] = ModulePredictControlWithAPF(vehicle, target_point, F_rep, max_speed, L, evader_pos, evader_virtual_point_speed)
    dt = 0.1; % 时间步长
    Kp = 3; % 比例增益
    
    % 预测虚拟质点的未来位置
    future_time = 0.3; % 预测0.3秒后的位置
    predicted_target = predictVirtualPointPosition(target_point, evader_pos, evader_virtual_point_speed, future_time);

    % 计算期望速度
    desired_velocity = Kp * (predicted_target - vehicle.position);
    
    % 添加避免与目标碰撞的斥力
    safe_distance = 0.3; % 安全距离
    d_evader = norm(vehicle.position - evader_pos);
    if d_evader < safe_distance
        F_evader = (1/d_evader - 1/safe_distance) * (1/d_evader^2) * (vehicle.position - evader_pos) / d_evader;
        F_rep = F_rep + F_evader;
    end

    % 添加人工势场力的影响
    desired_velocity = desired_velocity + F_rep;
    
    % 限制速度
    if norm(desired_velocity) > max_speed
        desired_velocity = max_speed * desired_velocity / norm(desired_velocity);
    end
    
    % 计算控制输入（加速度）
    optimal_control = (desired_velocity - vehicle.velocity) / dt;
    
    % 预测轨迹,相当于预留10个步长的滚动优化区间
    predicted_trajectory = zeros(10, 2);
    predicted_position = vehicle.position;
    predicted_velocity = vehicle.velocity;
    
    for i = 1:10
        predicted_velocity = predicted_velocity + optimal_control * dt;
        new_position = predicted_position + predicted_velocity * dt;
        [new_position, ~] = checkWallCollision(new_position, vehicle.length, vehicle.width, L);
        predicted_trajectory(i, :) = new_position(:)'; % 确保 new_position 是一个行向量
        predicted_position = new_position;
    end
    
    % 如果预测轨迹接近墙壁,调整控制输入
    if min(predicted_trajectory(:,1)) < 1 || max(predicted_trajectory(:,1)) > L-1 || ...
       min(predicted_trajectory(:,2)) < 1 || max(predicted_trajectory(:,2)) > L-1
        % 减小控制输入,使车辆远离墙壁
        optimal_control = optimal_control * 0.5;
    end
end



function vehicle = updateVehicle(vehicle, control, t_step, L)
    % 更新车辆状态
    vehicle.acceleration = control(:)';
    vehicle.velocity = vehicle.velocity(:)' + vehicle.acceleration * t_step;
    new_position = vehicle.position(:)' + vehicle.velocity * t_step;
    
    % 检查与墙壁的碰撞
    [new_position, collision] = checkWallCollision(new_position, vehicle.length, vehicle.width, L);
    
    if collision
        % 如果发生碰撞,减小速度
        vehicle.velocity = vehicle.velocity * 0.5;
    end
    
    vehicle.position = new_position;
    
    % 更新方向
    if norm(vehicle.velocity) > 0
        vehicle.orientation = atan2(vehicle.velocity(2), vehicle.velocity(1));
    end
end

%撞墙检测
function [new_position, collision] = checkWallCollision(position, length, width, L)
    collision = false;
    half_length = length / 2;
    half_width = width / 2;
    
    new_position = position(:)'; % 确保 new_position 是一个行向量
    
    % 检查x方向
    if new_position(1) - half_length < 0
        new_position(1) = half_length;
        collision = true;
    elseif new_position(1) + half_length > L
        new_position(1) = L - half_length;
        collision = true;
    end
    
    % 检查y方向
    if new_position(2) - half_width < 0
        new_position(2) = half_width;
        collision = true;
    elseif new_position(2) + half_width > L
        new_position(2) = L - half_width;
        collision = true;
    end
end


function pos = checkBounds(pos, L)
    % 确保位置在边界内
    pos = max(pos, 0);
    pos = min(pos, L);
end

function checkVehicleCollisions(vehicles, current_index, obstacles)
    current_vehicle = vehicles{current_index};
    
    % 检查与其他车辆的碰撞
    for i = 1:length(vehicles)
        if i ~= current_index
            if current_vehicle.checkCollision(vehicles{i})
                disp(['Collision detected between vehicle ', num2str(current_index), ' and vehicle ', num2str(i)]);
            end
        end
    end
    
    % 检查与障碍物的碰撞
    for i = 1:size(obstacles, 1)
        obstacle_vehicle = Vehicle(obstacles(i,:), [0,0], [0,0], 0, 0.2, 0.2); % 将障碍物视为静止的Vehicle对象
        if current_vehicle.checkCollision(obstacle_vehicle)
            disp(['Collision detected between vehicle ', num2str(current_index), ' and obstacle ', num2str(i)]);
        end
    end
end


function escape_direction = calculateEscapeDirection(evader_pos, pursuer_positions, obstacles, L,flag)
    % 计算逃离方向
    escape_direction = [0, 0];
    
    % 确保 pursuer_positions 是一个 Nx2 的矩阵
    pursuer_positions = reshape(pursuer_positions, [], 2);
    
    % 远离追捕者
    for i = 1:size(pursuer_positions, 1)
        diff = evader_pos - pursuer_positions(i, :);
        distance = norm(diff);
        escape_direction = escape_direction + diff / distance^2;
    end
    
    % 远离静态障碍物
    for i = 1:size(obstacles, 1)
        diff = evader_pos - obstacles(i, :);
        distance = norm(diff);
        if distance < 2 % 假设障碍物影响范围为2
            escape_direction = escape_direction + diff / distance^2;
        end
    end
    
    %考虑地图中心对逃避方向影响
    center = [L/2, L/2];
    to_center = center - evader_pos;
    center_distance = norm(to_center);

    % 仅对逃避者添加大量的向地图中心的吸引力
    if flag == 0
        center_force = to_center / center_distance^2;
        escape_direction = escape_direction + 5*center_force;
    else
        center_force = to_center / center_distance^2;
        escape_direction = escape_direction + 0.15*center_force;
    end
    
    % 添加切向力以鼓励循环运动
    tangential_force = [-to_center(2), to_center(1)];
    tangential_force = tangential_force / norm(tangential_force);
    escape_direction = escape_direction + tangential_force;
    
    % 添加远离墙壁的力
    wall_force = [0, 0];
    wall_range = 0.15; % 墙壁影响范围
    if evader_pos(1) < wall_range
        wall_force(1) = wall_range - evader_pos(1);
    elseif evader_pos(1) > L - wall_range
        wall_force(1) = L - wall_range - evader_pos(1);
    end
    if evader_pos(2) < wall_range
        wall_force(2) = wall_range - evader_pos(2);
    elseif evader_pos(2) > L - wall_range
        wall_force(2) = L - wall_range - evader_pos(2);
    end
    escape_direction = escape_direction + wall_force;
    
    % 归一化逃离方向
    if norm(escape_direction) > 0
        escape_direction = escape_direction / norm(escape_direction);
    else
        % 如果没有明确的逃离方向，随机选择一个方向
        angle = 2 * pi * rand();
        escape_direction = [cos(angle), sin(angle)];
    end
end


% 更新逃逸目标周围虚拟质点的函数
function virtual_points = updateEvaderVirtualPoints(center, num_points, radius, speed, time)
    angles = linspace(0, 2*pi, num_points+1);
    angles = angles(1:end-1);
    angles = angles + speed * time;
    x = cos(angles) * radius + center(1);
    y = sin(angles) * radius + center(2);
    virtual_points = [x; y]';
end

function predicted_point = predictVirtualPointPosition(current_point, center, speed, time)
    current_angle = atan2(current_point(2) - center(2), current_point(1) - center(1));
    future_angle = current_angle + speed * time;
    radius = norm(current_point - center);
    predicted_point = center + radius * [cos(future_angle), sin(future_angle)];
end

% 主程序结束
