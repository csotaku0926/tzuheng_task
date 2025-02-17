% function plot_two_fig(x, y, lgds, x_lim, x_ticks,...
%                         x2, y2, lgds2, x_lim2, x_ticks2, saved_filename)
%     % hb = barh(x, y); % (for bar horizontal)
%     hb = bar(x, y);

%     ylim(x_lim); %[50, 54]);
%     yticks(x_ticks); %50:0.5:54);
    
%     ylbl = "Update Cost Count (log 2)";
%     % xlabel("Number of Layers")
%     ylabel(ylbl);
%     xticklabels(["4 layers", "7 layers"]);
%     legend(lgds, 'Location', 'northeast');

%     nexttile
%     bar(x2, y2);
%     ylim(x_lim2);
%     yticks(x_ticks2);
%     legend(lgds2, 'Location', 'northeast');
    
%     exportgraphics(gcf, saved_filename);
% end

% Sample data
x1 = 1:4;  % X positions for first two columns
% y1 = [10 20 25 1;3 4 15 20;0 0 0 0]'; % First two columns (4 bars each)
y1 = zeros(4, 4);
y1(1:2, :) = transpose(readmatrix("saves/4-1_RW_search_space.csv"));
disp(y1);

% disp(size(y1));
x2 = x1;  % X positions for last column
M2 = readmatrix("saves/4-1_RW_rc_time.csv");
y2 = [0 0 log2(M2(1, 1)) + 8 0; 0 0 log2(M2(2, 1)) + 8 0; 0 0 log2(M2(3, 1)) + 8 0]'; % Last column (3 bars)
disp(y2);
% Create figure
figure;
% get current figure
ax = gca();
hold on;

% Left Y-axis (First two columns with 4 bars)
yyaxis left;
% change left ylabel color to black
ax.YAxis(1).Color = [0 0 0];
xticks(x1);
xticklabels(["4 layers", "7 layers", '1% failure', ''])
xtickangle(0);
b1 = bar(x1, y1, 'Facecolor', 'flat'); % Grouped bars

ylabel('Number of Visited Nodes');

clrs = {[.1, .1, .9], [.9, .1, .1], [.1, .9, .1], [.5, .1, .5], [.9, .9, .0]};
for k = 1:size(y1,1)
    b1(k).FaceColor = clrs{k};
end

% Right Y-axis (Last column with 3 bars)
yyaxis right;
% change right ylabel color to black
ax.YAxis(2).Color = [0 0 0];
b2 = bar(x2, y2); % Separate color for clarity
ylabel('Path Reconstruction Time (log 2 ms)');

% Labels and legend

for k = 1:size(y2,1) - 1
    b2(k).FaceColor = clrs{k};
end
b2(3).FaceColor = clrs{5};

legend([b1, b2(:, 3)], {'Dijkstra', 'A star', 'DBS_{OP}', 'DBS_{FS}', 'DBS_{RC}'},...
                         'Location', 'northeast');

hold off;

saved_filename = 'fig/pls_kill_me/RW_fig.jpg';
exportgraphics(gcf, saved_filename, "Resolution", 600);