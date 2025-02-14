clc
clear
addpath('utils\')

M_nlayer_lat = readmatrix('saves/1-1_nlayers_latency.csv');
M_nlayer_complexity = readmatrix('saves/1-1_nlayers_log2_complexity.csv');
M_density_lat = readmatrix('saves/1-2_density_latency.csv');
M_density_complexity = readmatrix('saves/1-2_density_log2_complexity.csv');
M_alt_lat = readmatrix('saves/1-2_alt_latency.csv');
M_alt_complexity = readmatrix('saves/1-2_alt_log2_complexity.csv');


nls = 1:3:n_layers;
alts = [500, 800, 1100, 1400, 1700];
dens = 1000:2000:5000;
algs = ["Dijkstra", "A star", "DBS-OP", "DBS-FS"];

for alg_i = 1:length(algs)
    fprintf("\\hline\n");
    fprintf("\\multirow{%d}{*}{%s} ", length(nls) + length(alts) + length(dens), algs(alg_i));

    alt = 1000;
    density = 1000;
    for nl = nls
        idx = nl;
        str = strcat("& ", num2str(nl), " & ", num2str(alt), " & ", num2str(density), " & ", ...
                num2str(M_nlayer_lat(alg_i, nl)), " & ", num2str(M_nlayer_complexity(alg_i, idx)));
        fprintf("%s \\\\ \n", str);
    end

    fprintf("\\cline{2-6}\n");
    % altitude scale
    nl = 4;
    density = 1000;

    for alt = 1:2:length(alts)
        idx = alt;
        str = strcat("& ", num2str(nl), " & ", num2str(alts(idx)), " & ", num2str(density), " & ", ...
            num2str(M_alt_lat(alg_i, nl)), " & ", num2str(M_alt_complexity(alg_i, idx)));
        fprintf("%s \\\\ \n", str);
    end

    fprintf("\\cline{2-6}\n");
    % node density
    alt = 600;
    nl = 4;
    for density = dens
        idx = density / 1000;
        str = strcat("& ", num2str(nl), " & ", num2str(alt), " & ", num2str(density), " & ", ...
            num2str(M_density_lat(alg_i, nl)), " & ", num2str(M_density_complexity(alg_i, idx)));
        fprintf("%s \\\\ \n", str);
    end
end