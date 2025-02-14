% 主程式
earth_radius = 6378; % 地球半徑（公里）
max_distance = 5016; % 每一段最長距離（公里）

% 生成衛星座標
sat_coords = generate_satellite_coordinates(24, 66, 53, 15, 5.45, 550);

% 定義地面站座標（紐約和雪梨）
ground_station_coords = [40.7128, -74.0060, 0; -33.8688, 151.2093, 0];
ground_station_coords = lla2ecef(ground_station_coords, earth_radius);

% 使用A*演算法尋找衛星路徑
[path, num_hops, path_length, path_lengths, elapsed_time] = find_path_A_star(sat_coords, ground_station_coords, earth_radius, max_distance);

% 文字輸出結果
fprintf('經過的hop數：%d\n', num_hops);
fprintf('每一段hop的路徑長度：');
fprintf('%.2f, ', path_lengths);
fprintf('\n總路徑長度：%.2f公里\n', path_length);
fprintf('A*演算法計算時間：%.4f秒\n', elapsed_time);

% 繪製3D路徑圖
plot_path_3D(sat_coords, ground_station_coords, path, earth_radius);

% 函數定義
% ...（在此處添加所有函數，包括generate_satellite_coordinates、lla2ecef、find_path_A_star、find_neighbors、h_score、reconstruct_path、calculate_path_lengths和plot_path_3D。）
% 生成衛星座標的函數
function sat_coords = generate_satellite_coordinates(num_orbits, num_sats_per_orbit, inclination, spacing_between, spacing_between_orbit_planes, altitude)
    num_sats = num_orbits * num_sats_per_orbit;
    sat_coords = zeros(num_sats, 3);
    earth_radius = 6378;
    orbit_radius = earth_radius + altitude;
    
    for i = 1:num_orbits
        for j = 1:num_sats_per_orbit
            sat_idx = (i - 1) * num_sats_per_orbit + j;
            theta = deg2rad((j - 1) * spacing_between);
            phi = deg2rad((i - 1) * spacing_between_orbit_planes);
            r = orbit_radius;
            
            sat_coords(sat_idx, :) = [r * cos(theta) * cos(phi), r * cos(theta) * sin(phi), r * sin(theta)];
        end
    end
    
    R = [1 0 0; 0 cosd(inclination) -sind(inclination); 0 sind(inclination) cosd(inclination)];
    sat_coords = (R * sat_coords')';
end

% 將緯度、經度、高度座標轉換為地球中心地固座標（ECEF）的函數
function ecef_coords = lla2ecef(lla_coords, earth_radius)

    num_coords = size(lla_coords, 1);
    ecef_coords = zeros(num_coords, 3);
    
    for i = 1:num_coords
        lat = deg2rad(lla_coords(i, 1));
        lon = deg2rad(lla_coords(i, 2));
        alt = lla_coords(i, 3);
        
        x = (earth_radius + alt) * cos(lat) * cos(lon);
        y = (earth_radius + alt) * cos(lat) * sin(lon);
        z = (earth_radius + alt) * sin(lat);
        
        ecef_coords(i, :) = [x, y, z];
    end
end

function [path, num_hops, path_length, path_lengths, elapsed_time] = find_path_A_star(sat_coords, ground_station_coords, earth_radius, max_distance)
    % 使用A*演算法找到最佳路徑的函數
    tic;
    
    start_node = ground_station_coords(1, :);
    end_node = ground_station_coords(2, :);
    all_nodes = [start_node; sat_coords; end_node];
    num_nodes = size(all_nodes, 1);
    
    g_score = inf(1, num_nodes);
    g_score(1) = 0;
    
    f_score = inf(1, num_nodes);
    f_score(1) = h_score(start_node, end_node, earth_radius);
    
    open_set = 1:num_nodes;
    
    came_from = zeros(1, num_nodes);
    
    while ~isempty(open_set)
        [~, idx] = min(f_score(open_set));
        current = open_set(idx);
        open_set(idx) = [];
        
        if current == num_nodes
            path = reconstruct_path(came_from, num_nodes);
            path_lengths = calculate_path_lengths(path, all_nodes, earth_radius);
            num_hops = length(path) - 1;
            path_length = sum(path_lengths);
            elapsed_time = toc;
            return;
        end
        
        neighbors = find_neighbors(current, all_nodes, earth_radius, max_distance);
        for neighbor = neighbors
            tentative_g_score = g_score(current) + h_score(all_nodes(current, :), all_nodes(neighbor, :), earth_radius);
            
            if tentative_g_score < g_score(neighbor)
                came_from(neighbor) = current;
                g_score(neighbor) = tentative_g_score;
                f_score(neighbor) = g_score(neighbor) + h_score(all_nodes(neighbor, :), end_node, earth_radius);
            end
        end
    end
    
    error('未能找到路徑');
end



% 找到節點鄰居的函數
function neighbors = find_neighbors(node_idx, all_nodes, earth_radius, max_distance)
    num_nodes = size(all_nodes, 1);
    neighbors = [];
    
    for i = 1:num_nodes
        if i == node_idx
            continue;
        end
        
        distance = h_score(all_nodes(node_idx, :), all_nodes(i, :), earth_radius);
        if distance <= max_distance
            neighbors = [neighbors, i];
        end
    end
end

function distance = h_score(node1, node2, earth_radius)
    % 計算兩個節點之間的大圓距離（h_score）的函數
    x1 = node1(1); y1 = node1(2); z1 = node1(3);
    x2 = node2(1); y2 = node2(2); z2 = node2(3);

    % 將笛卡爾坐標轉換為球面坐標
    r1 = sqrt(x1^2 + y1^2 + z1^2);
    r2 = sqrt(x2^2 + y2^2 + z2^2);
    lat1 = asin(z1 / r1);
    lon1 = atan2(y1, x1);
    lat2 = asin(z2 / r2);
    lon2 = atan2(y2, x2);

    delta_lat = lat2 - lat1;
    delta_lon = lon2 - lon1;

    a = sin(delta_lat / 2)^2 + cos(lat1) * cos(lat2) * sin(delta_lon / 2)^2;
    a = min(max(a, 0), 1);  % 限制 a 在 [0, 1]範圍內
    c = 2 * atan2(sqrt(a), max(1e-15, sqrt(1 - a))); % 避免除以零的情況

    distance = earth_radius * c;
end




function path = reconstruct_path(came_from, current)
    % 重建路徑的函數
    total_path = [current];
    while came_from(current) ~= 0
        current = came_from(current);
        total_path = [current, total_path];
    end
    path = total_path;
end

function path_lengths = calculate_path_lengths(path, all_nodes, earth_radius)
    % 計算路徑上每一段的長度的函數
    num_hops = length(path) - 1;
    path_lengths = zeros(1, num_hops);
    
    for i = 1:num_hops
        path_lengths(i) = h_score(all_nodes(path(i), :), all_nodes(path(i + 1), :), earth_radius);
    end
end

function plot_path_3D(sat_coords, ground_station_coords, path, earth_radius)
    % 繪製3D路徑的函數
    figure;
    hold on;
    
    % 繪製地球
    [X, Y, Z] = sphere(100);
    X = X * earth_radius;
    Y = Y * earth_radius;
    Z = Z * earth_radius;
    earth = surf(X, Y, Z);
    set(earth, 'FaceColor', [0.2, 0.5, 1.0], 'FaceAlpha', 0.5, 'EdgeColor', 'none');
    
    % 繪製衛星
    scatter3(sat_coords(:, 1), sat_coords(:, 2), sat_coords(:, 3), 20, 'ro', 'filled');
    
    % 繪製地面站
    scatter3(ground_station_coords(:, 1), ground_station_coords(:, 2), ground_station_coords(:, 3), 50, 'go', 'filled');
    text(ground_station_coords(1, 1), ground_station_coords(1, 2), ground_station_coords(1, 3), '紐約', 'FontSize', 12, 'FontWeight', 'bold');
    text(ground_station_coords(2, 1), ground_station_coords(2, 2), ground_station_coords(2, 3), '雪梨', 'FontSize', 12, 'FontWeight', 'bold');
    
    % 繪製路徑
    for i = 1:length(path) - 1
        if path(i) <= size(ground_station_coords, 1)
            node1 = ground_station_coords(path(i), :);
        else
            node1 = sat_coords(path(i) - size(ground_station_coords, 1), :);
        end
        if path(i + 1) <= size(ground_station_coords, 1)
            node2 = ground_station_coords(path(i + 1), :);
        else
            node2 = sat_coords(path(i + 1) - size(ground_station_coords, 1), :);
        end
        plot3([node1(1), node2(1)], [node1(2), node2(2)], [node1(3), node2(3)], 'k-', 'LineWidth', 1.5);
    end

end

