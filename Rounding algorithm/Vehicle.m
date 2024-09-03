classdef Vehicle
    properties
        position
        velocity
        acceleration
        orientation
        length
        width
    end
    
    methods
        function obj = Vehicle(position, velocity, acceleration, orientation, length, width)
            obj.position = position(:)';  % 确保是 1x2 行向量
            obj.velocity = velocity(:)';  % 确保是 1x2 行向量
            obj.acceleration = acceleration(:)';  % 确保是 1x2 行向量
            obj.orientation = orientation;
            obj.length = length;
            obj.width = width;
        end
        
        function collision = checkCollision(obj, other_vehicle)
            % 简单的碰撞检测，可以根据需要改进
            distance = norm(obj.position - other_vehicle.position);
            collision = distance < (obj.length + other_vehicle.length) / 2;
        end
    end
end
