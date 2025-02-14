function output_summary(alg_name, elapsed_time, path, total_distance, n_visited)
    fprintf('%s 演算法花費時間: %.4f秒\n',alg_name, elapsed_time);
    fprintf('經過的衛星節點數（包含起始與終點）: %d\n', length(path));
    fprintf('總路徑距離: %.2f公里\n', total_distance);
    fprintf('number of visited node: %.d\n', n_visited);
    fprintf('\n');
end