function plot_figure(x, y, lgds, x_lim, x_ticks, saved_filename)
    % hb = barh(x, y); % (for bar horizontal)
    hb = bar(x, y);

    % xlabel(yl);
    % ylabel(xl);
    ylim(x_lim); %[50, 54]);
    yticks(x_ticks); %50:0.5:54);
    
    ylbl = "Number of Visited Nodes";
    % xlabel("Number of Layers")
    ylabel(ylbl);
    xticklabels(["layer 1+2+3+...+10"]);
    % xticklabels(["1 failure in path", "3 failures in path", "5 failures in path",...
    %              "1% node failure", "5% node failure", "10% node failure"]);
    legend(lgds, 'Location', 'northeast');
    
    n_bar = length(hb);
    % plot text to bar
    % plot_bar(hb, n_bar);
    % plot_bar(hb, 1);

    exportgraphics(gcf, saved_filename);
end


function plot_bar(hb, ck)
    n_bar = length(hb);
    xWidth = hb(ck).BarWidth;

    xtips = hb(ck).XEndPoints;
    ytips = hb(ck).YEndPoints;

    for i = 1:length(xtips)
        xtips(i) = xtips(i) - 0.5 * xWidth / n_bar;
    end
    for i = 1:length(ytips)
        if ytips(i) > 0
            ytips(i) = ytips(i) + 0.5;
        else
            ytips(i) = 0.5;
        end
    end

    label = string(round(hb(ck).YData, 2));
    text(xtips, ytips, label, 'FontSize', 10);
end