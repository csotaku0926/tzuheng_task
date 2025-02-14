function path = reconstruct_path_to_end(came_from, current_satellite)
    total_path = [current_satellite];
    while came_from(current_satellite) > 0
        current_satellite = came_from(current_satellite);
        total_path = [current_satellite; total_path];
    end
    path = total_path;
end