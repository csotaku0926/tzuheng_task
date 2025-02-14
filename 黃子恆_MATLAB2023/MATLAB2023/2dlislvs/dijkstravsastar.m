% 生成節點
num_nodes = 10000;
max_distance = 400;
x = randi([0, 10000], 1, num_nodes);
y = randi([0, 10000], 1, num_nodes);

% 計算距離矩陣
distance_matrix = zeros(num_nodes);
for i = 1:num_nodes
    for j = i+1:num_nodes
        d = sqrt((x(i)-x(j))^2 + (y(i)-y(j))^2);
        if d <= max_distance
            distance_matrix(i, j) = d;
            distance_matrix(j, i) = d;
        else
            distance_matrix(i, j) = Inf;
            distance_matrix(j, i) = Inf;
        end
    end
end

% Dijkstra 演算法
tic
start_node = 1;
end_node = num_nodes;
unvisited_nodes = 1:num_nodes;
distances_dijkstra = ones(1, num_nodes) * Inf;
previous_nodes_dijkstra = zeros(1, num_nodes);
distances_dijkstra(start_node) = 0;

while ~isempty(unvisited_nodes)
    [~, current_node_idx] = min(distances_dijkstra(unvisited_nodes));
    current_node = unvisited_nodes(current_node_idx);
    if current_node == end_node
        break;
    end
    unvisited_nodes(current_node_idx) = [];

    neighbors = find(distance_matrix(current_node, :) < Inf);
    for neighbor = neighbors
        alt_distance = distances_dijkstra(current_node) + distance_matrix(current_node, neighbor);
        if alt_distance < distances_dijkstra(neighbor)
            distances_dijkstra(neighbor) = alt_distance;
            previous_nodes_dijkstra(neighbor) = current_node;
        end
    end
end
dijkstra_time = toc;

% A* 演算法
tic
open_list = start_node;
closed_list = [];
distances_astar = ones(1, num_nodes) * Inf;
previous_nodes_astar = zeros(1, num_nodes);
distances_astar(start_node) = 0;

while ~isempty(open_list)
    [~, current_node_idx] = min(distances_astar(open_list) + heuristic(open_list, end_node, x, y));
    current_node = open_list(current_node_idx);
    if current_node == end_node
        break;
    end
    open_list(current_node_idx) = [];
    closed_list = [closed_list, current_node];

    neighbors = find(distance_matrix(current_node, :) < Inf);
    for neighbor = neighbors
        if ismember(neighbor, closed_list)
            continue;
        end
        alt_distance = distances_astar(current_node) + distance_matrix(current_node, neighbor);
        if ~ismember(neighbor, open_list)
            open_list = [open_list, neighbor];
        elseif alt_distance >= distances_astar(neighbor)
            continue;
        end
        distances_astar(neighbor) = alt_distance;
        previous_nodes_astar(neighbor) = current_node;
    end
end
astar_time = toc;

% 繪製路徑
if previous_nodes_dijkstra(end_node) == 0 && previous_nodes_astar(end_node) == 0
    fprintf('從起始點到終點沒有可行的路徑。\n');
else
    fprintf('Dijkstra 演算法總距離為 %.2f km\n', distances_dijkstra(end_node));
    fprintf('A* 演算法總距離為 %.2f km\n', distances_astar(end_node));

    if distances_dijkstra(end_node) == distances_astar(end_node)
        fprintf('Dijkstra 和 A* 演算法找到的路徑距離相同。\n');
    elseif distances_dijkstra(end_node) < distances_astar(end_node)
        fprintf('Dijkstra 演算法找到的路徑比 A* 演算法短。\n');
    else
        fprintf('A* 演算法找到的路徑比 Dijkstra 演算法短。\n');
    end

    fprintf('Dijkstra 演算法運行時間：%.4f 秒\n', dijkstra_time);
    fprintf('A* 演算法運行時間：%.4f 秒\n', astar_time);

    % 重建路徑
    path_dijkstra = [end_node];
    while path_dijkstra(1) ~= start_node
        path_dijkstra = [previous_nodes_dijkstra(path_dijkstra(1)), path_dijkstra];
    end

    path_astar = [end_node];
    while path_astar(1) ~= start_node
        path_astar = [previous_nodes_astar(path_astar(1)), path_astar];
    end

    % 繪製圖形
    figure;
    scatter(x, y, 'filled');
    hold on;
    for i = 1:length(path_dijkstra)-1
        node1 = path_dijkstra(i);
        node2 = path_dijkstra(i+1);
        segment_distance = distance_matrix(node1, node2);
        plot([x(node1), x(node2)], [y(node1), y(node2)], 'r', 'LineWidth', 2);
        text((x(node1)+x(node2))/2, (y(node1)+y(node2))/2, sprintf('%.2f km', segment_distance), 'FontSize', 8);
    end
    for i = 1:length(path_astar)-1
        node1 = path_astar(i);
        node2 = path_astar(i+1);
        if ~ismember(node2, path_dijkstra)
            segment_distance = distance_matrix(node1, node2);
            plot([x(node1), x(node2)], [y(node1), y(node2)], 'b', 'LineWidth', 2);
            text((x(node1)+x(node2))/2, (y(node1)+y(node2))/2, sprintf('%.2f km', segment_distance), 'FontSize', 8);
        end
    end
    xlim([0, 10000]);
    ylim([0, 10000]);
    xlabel('X');
    ylabel('Y');
    title('最短路徑');
end

% 啟發式函數
function h = heuristic(node1, node2, x, y)
h = sqrt((x(node1)-x(node2)).^2 + (y(node1)-y(node2)).^2);
end