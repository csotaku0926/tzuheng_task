function plot_line(x, y, lgds, xlbl, ylbl, tick_lbl)
    % such as x = 10:10:100
    figure();
    hold on
    xticks(x);
    xlabel(xlbl);
    ylabel(ylbl);

    N_line = size(y, 1);
    for k = 1:N_line
        plot(x, y(k, :), 'Marker','o', 'DisplayName', lgds(k));
    end
    
    hold off

    legend(lgds, 'Location', 'best');
    xticklabels(tick_lbl);

end