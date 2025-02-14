% Dijkstra輔助函數
function [path, total_distance, elapsed_time] = Dijkstra_algorithm(satellite_coords, start_satellite, end_satellite, total_satellites, max_dis_perhop)
    tic; % 開始計時
    unvisited = 1:total_satellites;
    visited = [];
    
    distance = inf(1, total_satellites);
    distance(start_satellite) = 0;
    
    previous = zeros(1, total_satellites);
    
    while ~isempty(unvisited)
        [~, idx] = min(distance(unvisited));
        current_satellite = unvisited(idx);
        
        if current_satellite == end_satellite
            path = reconstruct_path(previous, current_satellite);
            total_distance = distance(current_satellite);
%            elapsed_time = toc; % 結束計時
            return;
        end
        
        unvisited(idx) = [];
        visited = [visited, current_satellite];
        
        neighbors = find_neighbors(satellite_coords, current_satellite, max_dis_perhop);
        for i = 1:length(neighbors)
            neighbor = neighbors(i);
            if ismember(neighbor, visited)
                continue;
            end
            
            alt = distance(current_satellite) + distance_between(satellite_coords, current_satellite, neighbor);
            if alt < distance(neighbor)
                distance(neighbor) = alt;
                previous(neighbor) = current_satellite;
            end
        end
        elapsed_time = toc; % 結束計時
    end
    
    % 若未找到路徑
    path = [];
    total_distance = 0;
    elapsed_time = toc;
end