clc
clear
addpath('utils\')

M_nlayer_lat = readmatrix('saves/1-1_nlayers_latency.csv');

M_FS_time = readmatrix('saves/Fig12_nlayer_time.csv');
M_batt_time = readmatrix('saves/3-1_outBattery_time.csv');

M_RC_time = readmatrix('saves/RC_time.csv');
M_RC_ratio_time = readmatrix('saves/RC_ratio_time.csv');
M_RC_time = log2(M_RC_time);
M_RC_ratio_time = log2(M_RC_ratio_time);

M_RW_update_cnt = readmatrix("saves/4-1_RW_search_space.csv");
M_RW_rc_time = readmatrix("saves/4-1_RW_rc_time.csv");


% plot figure
% n_layers = 10; % for 1-1, 2-1
% x_plot_axis = 1:2:n_layers;
% x_plot_axis = [500, 800, 1100, 1400, 1700]; % 1-2 alt
% x_plot_axis = 1000:1000:5000; % 1-2 density
% x_plot_axis = 1:1:5; % 1-2 inc


% x_plot_axis = [0.2, 0.3, 0.4, 0.5]; % 3-1 battery-drained node ratio
% x_tick_lbl = string(x_plot_axis);

x_plot_axis = 1:1;

xs = ["Dijkstra", "A star", "DBS_{OP}", "DBS_{FS}"];

M_FS_time = log2(M_FS_time);
M_RW_update_cnt = log2(M_RW_update_cnt);

% y_plot = [M_RC_time, M_RC_ratio_time] + 9;

y_plot = zeros(length(xs), 1);
y_plot(:, 1) = [21021, 19783, 12948, 9400];

% y_plot = zeros(length(xs), length(x_plot_axis));
% for i=1:length(x_plot_axis)
%     y_plot(:, i) = M_FS_time(:, x_plot_axis(i));
% end

x1 = 9000; x2 = 22000;
plot_figure(x_plot_axis, y_plot, xs, [x1 x2], x1:1000:x2, 'fig/pls_kill_me/FS_visited.jpg');
