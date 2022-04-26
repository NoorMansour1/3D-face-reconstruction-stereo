function [im_noB] = BackgroundRemove(im)
% Using Color Threshold to create mask for the image im1 and im2
[BWmask,maskedRGBImage1] = createMask(im); 

%Visualizing mask
Mask=bwareafilt(BWmask,1);
Mask=imfill(Mask,'holes');

% Applying mask
im_noB=im.*double(Mask);

figure(1);
subplot(1,3,1); imshow(BWmask)
subplot(1,3,2); imshow(Mask)
subplot(1,3,3); imshow(im_noB)
end