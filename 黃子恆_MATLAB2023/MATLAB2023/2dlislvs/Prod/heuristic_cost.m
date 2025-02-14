% 啟發式函數，點與點之間直線連接使用歐式距離

function cost = heuristic_cost(satellite_coords, current_satellite, end_satellite)
    cost = norm(satellite_coords(current_satellite, 1:3) - satellite_coords(end_satellite, 1:3));
end

%{
% 啟發式函數，點與點之間大圓距離
function cost = heuristic_cost(satellite_coords, current_satellite, end_satellite)
    % 地球半徑+衛星軌道高度（公里）
    R = 6378 + 540; 
    
    % 將座標轉換為經緯度（弧度）
    current_coords = satellite_coords(current_satellite, 1:3);
    [lat1, lon1, ~] = cart2sph(current_coords(1), current_coords(2), current_coords(3));
    
    end_coords = satellite_coords(end_satellite, 1:3);
    [lat2, lon2, ~] = cart2sph(end_coords(1), end_coords(2), end_coords(3));
    
    % 使用大圓距離公式計算距離
    delta_sigma = acos(sin(lat1) * sin(lat2) + cos(lat1) * cos(lat2) * cos(abs(lon2-lon1)));
    cost = R * delta_sigma;
end
%}