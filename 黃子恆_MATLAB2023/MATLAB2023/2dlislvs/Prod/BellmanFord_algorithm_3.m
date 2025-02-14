function [path, total_distance, elapsed_time] = Bellman_Ford_algorithm(edges, start_node, end_node, total_satellites)
    tic;
    
    % 初始化距離矩陣和前趨矩陣
    distance = inf(total_satellites, 1);
    predecessor = zeros(total_satellites, 1);
    
    distance(start_node) = 0;
    
    for i = 1:total_satellites-1
        for j = 1:size(edges, 1)
            u = edges(j, 1);
            v = edges(j, 2);
            w = edges(j, 3);
            if distance(u) + w < distance(v)
                distance(v) = distance(u) + w;
                predecessor(v) = u;
            end
        end
    end
    
    % 檢查是否存在負權重迴圈
    for i = 1:size(edges, 1)
        u = edges(i, 1);
        v = edges(i, 2);
        w = edges(i, 3);
        if distance(u) + w < distance(v)
            error('Graph contains a negative-weight cycle');
        end
    end
    
    % 從後向前回溯，找出最短路徑
    path = [];
    node = end_node;
    while node ~= start_node
        path = [node; path];
        node = predecessor(node);
    end
    path = [start_node; path];
    
    total_distance = distance(end_node);
    in this batch (to save space, only the last 50 messages in each batch are displayed)
    elapsed_time = toc;
end
