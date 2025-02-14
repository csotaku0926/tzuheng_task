% 地理座標轉換為直角座標
function [x, y, z] = llh2xyz(lat, lon, alt, earth_radius)
    lat_rad = deg2rad(lat);
    lon_rad = deg2rad(lon);
    x = (earth_radius + alt) * cos(lat_rad) * cos(lon_rad);
    y = (earth_radius + alt) * cos(lat_rad) * sin(lon_rad);
    z = (earth_radius + alt) * sin(lat_rad);
end