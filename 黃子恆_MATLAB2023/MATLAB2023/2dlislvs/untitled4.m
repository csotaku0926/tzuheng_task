% 初始化參數
clc; clear all; close all;

earth_radius = 6378; % 地球半徑 (km)
num_satellites = 1584; % 衛星總數
num_orbits = 24; % 軌道數
satellites_per_orbit = 66; % 每條軌道的衛星數
inclination = 53; % 軌道傾角 (degrees)
spacing_between = 15; % 衛星間距 (degrees)
spacing_between_orbit_plane = 5.45; % 軌道面間距 (degrees)
altitude = 550; % 衛星海拔高度 (km)

% 生成衛星座標
sat_coords = generate_satellite_coordinates(earth_radius, num_orbits, satellites_per_orbit, inclination, spacing_between, spacing_between_orbit_plane, altitude);

% 定義地面站座標 (紐約和雪梨)
ground_station_coords = [40.7128, -74.0060, 0; -33.8688, 151.2093, 0];
ground_station_coords = lla2ecef(ground_station_coords, earth_radius);

% 使用A*演算法找到最佳路徑
tic;
%[path, num_hops, path_length, path_lengths] = find_path_A_star(sat_coords, ground_station_coords, earth_radius);
[path, num_hops, path_length, path_lengths] = find_path_Dijkstra(sat_coords, ground_station_coords, earth_radius);

elapsed_time = toc;

% 顯示結果
fprintf('總共經過 %d 個 hop。\n', num_hops);
fprintf('每一段 hop 路徑長度：\n');
disp(path_lengths);
fprintf('總路徑長度：%.2f km\n', sum(path_lengths));
fprintf('A* 演算法計算時間：%.2f 秒\n', elapsed_time);

% 繪製3D路徑圖
plot_path_3D(sat_coords, ground_station_coords, path, earth_radius);

function sat_coords = generate_satellite_coordinates(earth_radius, num_orbits, satellites_per_orbit, inclination, spacing_between, spacing_between_orbit_plane, altitude)
% 生成衛星座標的函數
sat_coords = zeros(num_orbits * satellites_per_orbit, 3);
index = 1;

for i = 1:num_orbits
    for j = 1:satellites_per_orbit
        lat = asind(sind(inclination) * sind(spacing_between_orbit_plane * (i - 1)));
        lon = mod((j - 1) * spacing_between + (i - 1) * spacing_between / 2, 360);
        alt = earth_radius + altitude;
        sat_coords(index, :) = lla2ecef([lat, lon, alt], earth_radius);
        index = index + 1;
    end
end
end

function ecef = lla2ecef(lla, earth_radius)
% 將緯度、經度、高度座標轉換為地心地固座標的函數
lat = deg2rad(lla(:, 1));
lon = deg2rad(lla(:, 2));
alt = lla(:, 3);
N = earth_radius ./ sqrt(1 - (0.0818191908426 .* sin(lat)).^2);
x = (N + alt) .* cos(lat) .* cos(lon);
y = (N + alt) .* cos(lat) .* sin(lon);
z = ((1 - 0.0818191908426^2) .* N + alt) .* sin(lat);

ecef = [x, y, z];
end

function [path, num_hops, path_length, path_lengths] = find_path_A_star(sat_coords, ground_station_coords, earth_radius)
% 使用A*演算法找到最佳路徑的函數
start_node = ground_station_coords(1, :);
end_node = ground_station_coords(2, :);
all_nodes = [start_node; sat_coords; end_node];
num_nodes = size(all_nodes, 1);

open_list = [1];
closed_list = [];

g_scores = inf(1, num_nodes);
g_scores(1) = 0;

f_scores = inf(1, num_nodes);
f_scores(1) = h_score(start_node, end_node, earth_radius);

came_from = zeros(1, num_nodes);

while ~isempty(open_list)
    [~, idx] = min(f_scores(open_list));
    current = open_list(idx);
    open_list(idx) = [];

    if current == num_nodes
        path = reconstruct_path(came_from, num_nodes);
        path_lengths = calculate_path_lengths(path, all_nodes, earth_radius);
        num_hops = length(path) - 1;
        path_length = sum(path_lengths);
        return;
    end

    closed_list = [closed_list, current];

    neighbors = find_neighbors(current, all_nodes, earth_radius);
    for neighbor = neighbors
        if ismember(neighbor, closed_list)
            continue;
        end

        tentative_g_score = g_scores(current) + h_score(all_nodes(current, :), all_nodes(neighbor, :), earth_radius);
        if ~ismember(neighbor, open_list)
            open_list = [open_list, neighbor];
        elseif tentative_g_score >= g_scores(neighbor)
            continue;
        end

        came_from(neighbor) = current;
        g_scores(neighbor) = tentative_g_score;
        f_scores(neighbor) = g_scores(neighbor) + h_score(all_nodes(neighbor, :), end_node, earth_radius);
    end
end

error('未能找到路徑');
end

function neighbors = find_neighbors(node, all_nodes, earth_radius)
% 找到節點的相鄰節點的函數
distances = vecnorm(all_nodes - all_nodes(node, :), 2, 2);
neighbors = find((distances > 0) & (distances <= 5016));

end

function h = h_score(node1, node2, earth_radius)
% 計算節點間的距離的函數
h = norm(node1 - node2) / 1000;
end

function path = reconstruct_path(came_from, current)
% 重建路徑的函數
total_path = [current];
while current ~= 1
    current = came_from(current);
    total_path = [current, total_path];
end
path = total_path;
end

function path_lengths = calculate_path_lengths(path,all_nodes, earth_radius)
% 計算每一段路徑長度的函數
path_lengths = zeros(1, length(path) - 1);
for i = 1:length(path) - 1
    path_lengths(i) = h_score(all_nodes(path(i), :), all_nodes(path(i + 1), :), earth_radius);
end
end

function [path, num_hops, path_length, path_lengths] = find_path_Dijkstra(sat_coords, ground_station_coords, earth_radius)
    % 使用Dijkstra演算法找到最佳路徑的函數

    start_node = ground_station_coords(1, :);
    end_node = ground_station_coords(2, :);
    all_nodes = [start_node; sat_coords; end_node];
    num_nodes = size(all_nodes, 1);

    dist = inf(1, num_nodes);
    dist(1) = 0;

    unvisited = 1:num_nodes;

    came_from = zeros(1, num_nodes);

    while ~isempty(unvisited)
        [~, idx] = min(dist(unvisited));
        current = unvisited(idx);
        unvisited(idx) = [];

        if current == num_nodes
            path = reconstruct_path(came_from, num_nodes);
            path_lengths = calculate_path_lengths(path, all_nodes, earth_radius);
            num_hops = length(path) - 1;
            path_length = sum(path_lengths);
            return;
        end

        neighbors = find_neighbors(current, all_nodes, earth_radius);
        for neighbor = neighbors
            alt = dist(current) + h_score(all_nodes(current, :), all_nodes(neighbor, :), earth_radius);
            if alt < dist(neighbor)
                dist(neighbor) = alt;
                came_from(neighbor) = current;
            end
        end
    end

    error('未能找到路徑');
end


function plot_path_3D(sat_coords, ground_station_coords, path, earth_radius)
% 繪製3D路徑圖的函數
all_nodes = [ground_station_coords(1, :); sat_coords; ground_station_coords(2, :)];

% 繪製地球
[x, y, z] = sphere(50);
x = x * earth_radius;
y = y * earth_radius;
z = z * earth_radius;
figure;
surf(x, y, z, 'EdgeColor', 'none', 'FaceColor', [0.8, 0.8, 0.8], 'FaceAlpha', 0.5);
hold on;

% 繪製衛星節點
scatter3(sat_coords(:, 1), sat_coords(:, 2), sat_coords(:, 3), 5, 'b', 'filled');
hold on;

% 繪製地面站節點
scatter3(ground_station_coords(:, 1), ground_station_coords(:, 2), ground_station_coords(:, 3), 20, 'r', 'filled');
text(ground_station_coords(1, 1), ground_station_coords(1, 2), ground_station_coords(1, 3), '紐約', 'FontSize', 8);
text(ground_station_coords(2, 1), ground_station_coords(2, 2), ground_station_coords(2, 3), '雪梨', 'FontSize', 8);

% 繪製路徑
for i = 1:length(path) - 1
    x = [all_nodes(path(i), 1), all_nodes(path(i + 1), 1)];
    y = [all_nodes(path(i), 2), all_nodes(path(i + 1), 2)];
    z = [all_nodes(path(i), 3), all_nodes(path(i + 1), 3)];
    plot3(x, y, z, 'r', 'LineWidth', 1);
end

% 設置圖形標題和軸標籤
title('衛星路徑');
xlabel('X');
ylabel('Y');
zlabel('Z');

% 設置視圖
view(3);
grid on;
axis equal;
end