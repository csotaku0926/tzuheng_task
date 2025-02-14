% HA* 適應性啟發式函數
function cost = adaptive_heuristic_cost(satellite_coords, current_satellite, end_satellite, g_cost, learning_rate)
    % 計算歐式距離
    estimated_cost = norm(satellite_coords(current_satellite, :) - satellite_coords(end_satellite, :));

    % 根據學習因子和已經探索到的真實距離更新啟發函數
    cost = estimated_cost + learning_rate * (g_cost(current_satellite) - estimated_cost);
end
