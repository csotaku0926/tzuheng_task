clc
clear
addpath('utils\')

M_gamma_latency = readmatrix('saves/2-2_gamma_latency.csv');
M_gamma_search_space = readmatrix('saves/2-2_gamma_search_space.csv');

gammas = [1e3, 1e4, 1e5, 1e6, 1e7]; 
algs = ["Dijkstra", "A star", "DBS-OP", "DBS-FS"];

% for alg_i = 1:length(algs)
%     fprintf("\\hline\n");
%     fprintf("\\multirow{%d}{*}{%s} ", length(gammas), algs(alg_i));

%     for gamma_i = 1:length(gammas)
%         lat_i = M_gamma_latency(alg_i, gamma_i);
%         search_space_i = M_gamma_search_space(alg_i, gamma_i);

%         str = strcat("& ", sci_notation(gammas(gamma_i)), " & ", num2str(lat_i), " & ", num2str(search_space_i));
%         fprintf("%s \\\\ \n", str);
%     end
% end

% fprintf("\\hline\n");

n_layers = 10;
min_alt = 500; max_alt = 3000;
dalt = round((max_alt - min_alt) / n_layers);
alts = min_alt:dalt:max_alt;

for i=1:10
    fprintf("\\hline\n");
    str = strcat(num2str(alts(i)), " & 2000 & 200 & 0°");
    fprintf("%s \\\\ \n", str);
end


fprintf("\\hline\n");