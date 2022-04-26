function [im_noB,foreground] = BackgroundRemoveKmean(im)
% Remove the background of the picture using k mean
% The picture can be divided into 3 groups using k mean: Background
% (Brightest-1), Skin (Medium-2), Eye-Shirt (Darkest-3) 
% Thus choose the background and eye-shirt to remove
% - Keep the eye by using filling hole functions
% - Remove small details

% Create a save place
[m,n,x] = size(im);
foreground = zeros(m,n);

% Dividing into clusters
% k-means to divide picture into 3 clusters
pixel_labels = k_means(im,3);                         
% Checking which cluster is background (1) and shirt (3)
f1=figure;
imshow(im)
disp('Choose a point of background and a point on the shirt')
[x,y]=getpts;
close(f1)
% Remove the background and a shirt
foreground(:,:) = (pixel_labels ~= pixel_labels(round(y(1)),round(x(1)))&pixel_labels ~=pixel_labels(round(y(2)),round(x(2))));

% Finishing
% Refill the eyes
closed = imclose(foreground, strel('disk', 13));
foreground = imfill(closed, 'holes');
%Deleting small details
SE = strel('line',11,0);
seed = imerode(foreground,SE);
foreground = imreconstruct(seed,foreground);
%figure();imshow(foreground_labels,[]);title('Foreground Mask');

% Masking foreground out of the origininal picture
im_noB=im.*double(foreground);
end