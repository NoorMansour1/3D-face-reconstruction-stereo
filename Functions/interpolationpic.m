function disparitymap= interpolationpic(disparitymap)
%% Filling missing pixels in the disparity map --interpolation
w=10;
mask=fspecial('gauss',2*w+1,5);
[M,N]=size(disparitymap);
for i=1:M
    for j=1:N
        if disparitymap(i,j)~=0 && ~isnan(disparitymap(i,j))
            win = disparitymap(i-w:i+w,j-w:j+w);
            nonZeros= win~=0;
            win=mask.*win;
            if sum(sum(mask.*nonZeros)) ~=0
                disparitymap(i,j) = sum(sum(win.*nonZeros))/sum(sum(mask.*nonZeros));
            end
        end
    end      
end
end