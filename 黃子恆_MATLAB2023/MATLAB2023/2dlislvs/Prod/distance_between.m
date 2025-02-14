% 當前衛星節點與其相連鄰居節點距離
function distance = distance_between(satellite_coords, current_satellite, neighbor)
    distance = norm(satellite_coords(current_satellite, 1:3) - satellite_coords(neighbor, 1:3));
end