function max_dis = calc_max_dis(h1, h2)
    earth_radius = 6378; % 地球半徑(km)
    atm_high = 80; % 含水氣之大氣層海拔高度(km)
    max_dis = floor(sqrt((earth_radius + h1)^2 - (earth_radius + atm_high)^2) + sqrt((earth_radius + h2)^2 - (earth_radius + atm_high)^2));
end

%{
max_dis_540_540 = calc_max_dis(540, 540);
max_dis_550_550 = calc_max_dis(550, 550);
max_dis_560_560 = calc_max_dis(560, 560);
max_dis_570_570 = calc_max_dis(570, 570);
max_dis_540_550 = calc_max_dis(540, 550);
max_dis_540_560 = calc_max_dis(540, 560);
max_dis_540_570 = calc_max_dis(540, 570);
max_dis_550_560 = calc_max_dis(550, 560);
max_dis_550_570 = calc_max_dis(550, 570);
max_dis_560_570 = calc_max_dis(560, 570);
%}