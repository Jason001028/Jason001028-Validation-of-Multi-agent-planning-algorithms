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
    
    % 结合APF力和障碍物
    combined_direction = optimal_direction + F_rep;
    if norm(combined_direction) > 0
        combined_direction = combined_direction / norm(combined_direction);
    end
    
    optimal_control = combined_direction * max_speed;
end