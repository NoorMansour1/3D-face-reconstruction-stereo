function [mean_angles, rms_angles] = ut_check_rect(varargin)
%[mean_angles,rms_angles] = ut_check_rect(I1,I2,[visualize])
%
% ut_check_rect checks whether stereo rectification was successful. It
% first estimates the fundamental matrix of two stereo images
% I1 and I2 by detecting SURF key points in both images. These key points
% are matched to create a set of corresponding pair of points. This set is 
% used to estimate the fundamental matrix F.
%
% Of each displacement vector of each corresponding pair, the
% angle is calculated wrt the horizontal axis. The mean and rms of these
% angels are estimates so as to provide a measure of rectification. In
% case of a perfect rectification, all angles should be zero. Angles are
% expressed in degrees.
%
% With the option 'visualize' equal to true, a figure will be created with the images
% and the corresponding points.

[I1,I2,isvisualize] = ParseInputs(varargin{:});

%% find SURF key points in both images, and extract their descriptors (features)
blobs1 = detectSURFFeatures(rgb2gray(I1), 'MetricThreshold', 5);
blobs2 = detectSURFFeatures(rgb2gray(I2), 'MetricThreshold', 5);
[features1, validBlobs1] = extractFeatures(rgb2gray(I1), blobs1);
[features2, validBlobs2] = extractFeatures(rgb2gray(I2), blobs2);

%% match features
indexPairs = matchFeatures(features1, features2, 'Metric', 'SAD', 'MatchThreshold', 50);
matchedPoints1 = validBlobs1(indexPairs(:,1),:);
matchedPoints2 = validBlobs2(indexPairs(:,2),:);

%% robustly estimate fundamental matrix 
[F, inliers] = estimateFundamentalMatrix(matchedPoints1,matchedPoints2,'NumTrials',2000);
matchedPoints1 = matchedPoints1(inliers);
matchedPoints2 = matchedPoints2(inliers);

%% get the angles of connecting lines in degress
P = matchedPoints1.Location;
Q = matchedPoints2.Location;
PQ = (P-Q);
angles = atan(PQ(:,2)./PQ(:,1))*180/pi;
rms_angles = rms(angles);
mean_angles = mean(angles);


%% visualize
if isvisualize
    figure(101)
    showMatchedFeatures(I1, I2, matchedPoints1, matchedPoints2,'blend','PlotOptions',{'bo','ro','k-'});
    axis on
    legend('Matched points in I1', 'Matched points in I2');
    title(['minmimum disparity: ',num2str(min(PQ(:,1))),'   maximum disparity: ',num2str(max((PQ(:,1))))])
    % draw some epipolar lines
    % x = [1 ncol];
    % hold on;
    % for i=1:length(matchedPoints1)
    %     a = F*[P(i,:), 1]';
    %     y = [(-a(1)*x(1)-a(3))/a(2), (-a(1)*x(2)-a(3))/a(2)];
    %     plot(x,y,'r');
    %     plot(P(i,1),P(i,2),'b.')
    %
    %     b = F'*[Q(i,:), 1]';
    %     y = [(-b(1)*x(1)-b(3))/b(2), (-b(1)*x(2)-b(3))/b(2)];
    % %     plot(x,y,'b');
    %     plot(Q(i,1),Q(i,2),'r.')
    % end
    
end
end

function y = rms(x)             % rms is a function in the signal processing toolbox
y = sqrt(mean(x .* x));
end


%----------------------------------------------------------------------
% Subfunction ParseInputs
%----------------------------------------------------------------------

function [I1,I2,isvisualize] = ParseInputs(varargin)

narginchk(2,3);
if nargin==3, isvisualize = true; else isvisualize=false; end
I1 = (varargin{1});
I2 = (varargin{2});
if ndims(I1)~=3, error('input must be NxMx3 image'); end;
end

