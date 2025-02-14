function values = normal_distribution(mean, sigma, n_item, min_value)
    values = max(mean + randn(n_item, 1) * sigma, min_value);
end