% A*輔助函數
function [path, total_distance, elapsed_time] = A_star_algorithm(satellite_coords, start_satellite, end_satellite, total_satellites, max_dis_perhop)

    tic; % 開始計時
    open_list = [start_satellite];
    closed_list = [];
    
    g_cost = zeros(total_satellites, 1);
    h_cost = zeros(total_satellites, 1);
    f_cost = zeros(total_satellites, 1);
    came_from = zeros(total_satellites, 1);
    
    g_cost(start_satellite) = 0;
    h_cost(start_satellite) = heuristic_cost(satellite_coords, start_satellite, end_satellite);
    f_cost(start_satellite) = g_cost(start_satellite) + h_cost(start_satellite);
    
    while ~isempty(open_list)
        % 從open_list中找到f_cost最小的節點
        [~, min_index] = min(f_cost(open_list));
        current_satellite = open_list(min_index);
        
        if current_satellite == end_satellite
            path = reconstruct_path(came_from, current_satellite);
            total_distance = g_cost(current_satellite);
%            elapsed_time = toc; % 結束計時
            return;
        end
        
        open_list(min_index) = [];
        closed_list = [closed_list; current_satellite];
        
        % 遍歷當前節點的鄰居節點
        neighbors = find_neighbors(satellite_coords, current_satellite, max_dis_perhop);
        for i = 1:length(neighbors)
            neighbor = neighbors(i);
            if ismember(neighbor, closed_list)
                continue;
            end
            
            tentative_g_cost = g_cost(current_satellite) + distance_between(satellite_coords, current_satellite, neighbor);
            
            if ~ismember(neighbor, open_list)
                open_list = [open_list; neighbor];
            elseif tentative_g_cost >= g_cost(neighbor)
                continue;
            end
            
            came_from(neighbor) = current_satellite;
            g_cost(neighbor) = tentative_g_cost;
            h_cost(neighbor) = heuristic_cost(satellite_coords, neighbor, end_satellite);
            f_cost(neighbor) = g_cost(neighbor) + h_cost(neighbor);
        end
        elapsed_time = toc; % 結束計時
    end
    
    % 若未找到路徑
    path = [];
    total_distance = 0;
    elapsed_time = toc;
end