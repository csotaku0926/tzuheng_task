function notation = sci_notation(num)
    is_neg = (num < 0);
    num = abs(num);

    expo_count = 0;
    while num >= 10.0
        expo_count = expo_count + 1;
        num = num / 10.0;
    end

    notation = strcat(num2str(((-1) ^ is_neg) * num), 'e', int2str(expo_count));
end