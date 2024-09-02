classdef Vehicle
    properties
        position % 位置 [x, y]
        velocity % 速度 [vx, vy]
        acceleration % 加速度 [ax, ay]
        orientation % 方向角度 (弧度)
        length % 车辆长度
        width % 车辆宽度
    end
    
    methods
        function obj = Vehicle(position, velocity, acceleration, orientation, length, width)
            obj.position = position;
            obj.velocity = velocity;
            obj.acceleration = acceleration;
            obj.orientation = orientation;
            obj.length = length;
            obj.width = width;
        end
        
        function obj = update(obj, dt)
            % 更新速度
            obj.velocity = obj.velocity + obj.acceleration * dt;
            
            % 更新位置
            obj.position = obj.position + obj.velocity * dt;
            
            % 更新方向
            obj.orientation = obj.orientation + obj.velocity(1) * tan(obj.acceleration(2)) / obj.length * dt;
        end
        
        function collision = checkCollision(obj, other)
            % 碰撞检测
            dist = norm(obj.position - other.position);
            collision = dist < (obj.length + other.length) / 2;
        end
    end
end
