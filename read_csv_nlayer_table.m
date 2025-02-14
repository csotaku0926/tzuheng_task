clc
clear
addpath('utils\')

M_nlayer_lat = readmatrix('saves/T2_nlayer_latency.csv');
M_nlayer_complexity = readmatrix('saves/T2_nlayer_visited.csv');
M_regular_lat = readmatrix('saves/regularity_latency.csv');
M_regular_complexity = readmatrix('saves/regularity_visited.csv');

n_layers = size(M_nlayer_lat, 2);
n_regular = size(M_regular_lat, 2);
nls = [4];
algs = ["Dijkstra", "A star", "DBS-OP"];

config_lst = ["1+2+3", "4+5+6", "7+9+10"];

for alg_i = 1:length(algs)
    fprintf("\\hline\n");
    fprintf("\\multirow{%d}{*}{%s} ", length(nls) + n_regular - 1, algs(alg_i));

    for nl = nls
        idx = nl;
        str = strcat("& ", make_config_str(nl), " & ", ...
                num2str(M_nlayer_lat(alg_i, nl)), " & ", num2str(M_nlayer_complexity(alg_i, idx)));
        fprintf("%s \\\\ \n", str);
    end

    for i = 2:n_regular
        str = strcat("& ", config_lst(i), " & ", ...
            num2str(M_regular_lat(alg_i, i)), " & ", num2str(M_regular_complexity(alg_i, i)));
        fprintf("%s \\\\ \n", str);
    end

end


function config_str = make_config_str(n_layers)
    config_str = "1";
    i = 1;
    while (i < n_layers)
        i = i + 1;
        if (i == 4 && i < n_layers)
            config_str = strcat(config_str, "+...");
        elseif (i < 4 || i == n_layers)
            config_str = strcat(config_str, "+");        
            config_str = strcat(config_str, num2str(i));
        end
    end
end