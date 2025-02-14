% 搜尋到更短的路徑時，重建新路徑
function path = reconstruct_path(came_from, current_satellite)
    total_path = [current_satellite];
    while came_from(current_satellite) > 0
        current_satellite = came_from(current_satellite);
        total_path = [current_satellite; total_path];
    end
    path = total_path;
end