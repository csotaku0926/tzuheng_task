% 定義一個函數，這個函數會計算在某個時間點地面站的ECEF座標
function [x, y, z] = calculate_gs_coordinates(earth_radius, latitude, longitude, t)
    % 地球自轉角速度，單位是度/秒
    rotation_speed = 360 / 86400;

    % 計算當前時間地球已經旋轉了多少度
    rotation_angle = rotation_speed * t;

    % 計算新的經度
    new_longitude = longitude + rotation_angle;

    % 轉換經緯度座標為三維座標
    [x, y, z] = llh2xyz(latitude, new_longitude, 0, earth_radius);
end