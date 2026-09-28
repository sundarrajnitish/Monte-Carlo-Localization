function x_temp = FunctionLowVarianceResample(x)
    M = size(x, 1);
    x_temp = zeros(M, 4);
    r = rand / M;
    c = x(1, 4);
    i = 1;

    for m = 1:M
        U = r + (m - 1) / M;

        while U > c && i<M
            i = i + 1;
            c = c + x(i, 4);
        end

        x_temp(m, :) = x(i, :);
    end
end

