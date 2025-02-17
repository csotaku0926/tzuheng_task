clc
clear
addpath('utils\')

M_nlayer_lat = readmatrix('saves/table2_latency.csv');
M_nlayer_complexity = readmatrix('saves/table2_visited.csv');

nlayers_list = [1,3,4,6,8,9];
config_list = ["$\{4,5,6\}$ (irregular)", "$\{7,9,10\}$ (dense)",...
             "$\{5,6,7,8\}$ (irregular)", "$\{6,7,8,9\}$ (dense)"];
algs = ["Dijkstra", "A star", "DBS-OP"];

for nl_i = 1:length(nlayers_list)
    nl = nlayers_list(nl_i);
    fprintf("\\hline\n");
    fprintf("\\multirow{%d}{*}{%s}\n", length(algs), make_config_str(nl));
    
    for alg_i = 1:length(algs)
        str = strcat("& ", algs(alg_i), " & ", ...
                num2str(M_nlayer_lat(alg_i, nl_i)), " & ", num2str(M_nlayer_complexity(alg_i, nl_i)));
        fprintf("%s \\\\ \n", str);
    end
end

for config_i = 1:length(config_list)
    config = config_list(config_i);
    fprintf("\\hline\n");
    fprintf("\\multirow{%d}{*}{%s}\n", length(algs), config);
    
    for alg_i = 1:length(algs)
        str = strcat("& ", algs(alg_i), " & ", ...
                num2str(M_nlayer_lat(alg_i, length(nlayers_list) + config_i)), ...
                    " & ", num2str(M_nlayer_complexity(alg_i, length(nlayers_list) + config_i)));
        fprintf("%s \\\\ \n", str);
    end
end

function config_str = make_config_str(n_layers)
    config_str = "$\{1";
    i = 1;
    while (i < n_layers)
        i = i + 1;
        if (i == 4 && i < n_layers)
            config_str = strcat(config_str, ",\dots");
        elseif (i < 4 || i == n_layers)
            config_str = strcat(config_str, ",");        
            config_str = strcat(config_str, num2str(i));
        end
    end

    config_str = strcat(config_str, "\}$");
end