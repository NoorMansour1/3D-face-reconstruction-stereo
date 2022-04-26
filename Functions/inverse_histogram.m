function g = inverse_histogram(h, nbins)

g = h*0;
for j = 1:nbins
    tmp = find(h >= (j-1)/nbins, 1, 'first');
    if size(tmp,2) == 0
        tmp = g(j-1);
    end
    g(j) = tmp;
end

end