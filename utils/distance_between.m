function distance = distance_between(satellite_coords, current_satellite, neighbor)
    distance = norm(satellite_coords(current_satellite, :) - satellite_coords(neighbor, :));
end