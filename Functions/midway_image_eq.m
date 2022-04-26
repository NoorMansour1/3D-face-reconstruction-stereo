function [U, H_midway, H, H_inv, min_max] = midway_image_eq(V, min_max, save_images)
% Implementation of the Midway Equalization algorithm
% as described in Algorithm 1 of the paper:
%   Thierry Guillemot, and Julie Delon, 
%   Implementation of the Midway Image Equalization, 
%   Image Processing On Line, 6 (2016), pp. 114–129. 
%   https://doi.org/10.5201/ipol.2016.140
%
% Input:
%   V: collection of input images as a cell array
%   masks: cell array containing binary masks that defines the
%          pixels on which to calculate histograms. If empty,
%          uses all pixels
%
% Output:
%   U: collection of equalized images as a cell array
%   H_midway: midway histogram 
%   H: histogram of the input images
%   H_inv: inverse of H
%
% Author:
%   Antonio Stanziola; Biomedical Ultrasound Group, UCL - a.stanziola@ucl.ac.uk
maxval = 0;
for i = 1:length(V)
  v = V{i};
  max_this = ceil(max(reshape(v, 1, [])));
  if max_this > maxval
    maxval = max_this;
  end
  disp([i, maxval])
end
min_max(2) = maxval;
nbins = min_max(2)+1-min_max(1);
disp([min_max(1), min_max(2), nbins]);

% Making range masks
masks = cell(size(V));
for i = 1:length(V)
    m = ones(size(V{i}));
    m(V{i} < min_max(1)) = 0;
    m(V{i} > min_max(2)) = 0;
    masks{i} = logical(m);
end

if nargin < 3
    save_images = true;
end

% Compute the histogram of each image and store it
disp("Computing histograms")
h = cell(1, length(V));
edges = linspace(min_max(1), min_max(2)+1, nbins + 1) - 0.5;
for i = 1:length(V)
    volume = V{i};
    pixels = volume(masks{i});
    pixels = int32(pixels);

    numpix = numel(pixels);
    h{i} = histcounts(pixels, edges);
    h{i} = h{i}/numpix;
end

% Computes the cumulative histograms
disp("Computing cumulative histograms")
H = cell(1, length(V));
for i = 1:length(V)
    H{i} = cumsum(h{i});
end

% Find the inverse function of each histogram
disp("Computing inverse cumulative histograms")
H_inv = cell(1, length(V));
for i = 1:length(V)
    H_inv{i} = inverse_histogram(H{1}, nbins);
end

% Compute the midway histogram as the harmonic mean of
% the cumulative histograms
disp("Computing the midway histogram")
H_midway = H{1}*0;
for i = 1:length(V)
    H_midway = H_midway + H_inv{i};
end
H_midway = H_midway/length(V);

% Applies the inverse transform to each pixel of each image
U = cell(1, length(V));
for i = 1:length(V)
    disp(['Equalizing image ' num2str(i)])
    vol = V{i};
    U{i} = apply_midway_transform(V{i}, H_midway, min_max);

%     % Save images
%     if save_images
%         n = i;
%         disp(['Saving image ' num2str(n) ' of ' num2str(length(V))]);
%         figure;
%         set(gcf,'Position',[0 0 300 1000])
%         
%         subplot(2,2,1);
%         vol_ = U{n};
%         maxval = max(vol_(:));
%         imagesc(squeeze(vol_(90,:,:)))
%         caxis([min_max(1),maxval])
%         colormap gray
%         title('equalized') 
% 
%         subplot(2,2,3);
%         histogram(vol_)
%         set(gca, 'YScale', 'log')
%         title('matched hist')
% 
%         subplot(2,2,2);
%         vol_ = V{n};
%         imagesc(squeeze(vol_(90,:,:)))
%         caxis([min_max(1),maxval])
%         colormap gray
%         title('original')
%         
%         subplot(2,2,4);
%         histogram(vol_)
%         set(gca, 'YScale', 'log')
%         title('original hist')
%         
%         scaleFig(2, 3);
% 
%         drawnow
%         saveas(gcf,['./hist_eq/' num2str(n) '.png'])
%         close all
%     end
end


end