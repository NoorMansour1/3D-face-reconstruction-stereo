%% PROJECT 3: 3D Face Reconstruction 
% Participant: Noor Mansour, Luan Duong, Vincent van Engers
% Project description: Building 3D pointclouds and 3D mesh from 3 images at
% different angles
clear all;
close all;

% Load 3 pictures
imleft=imread('subject4_Left_1.jpg');
immid=imread('subject4_Middle_1.jpg');
imright=imread('subject4_Right_1.jpg');

%Convert uint8 to double
imleft = im2double(imleft);
immid = im2double(immid);
imright = im2double(imright);

% Add path
addpath("Functions\"); 

%% Task 1: Pre-processing - Obtain intrinsic and the external parameters of the camera from  camera  calibration using checker boards
% - Use the STEREO camera calibration app in Matlab to select chessboard images
% - The size of chessboard is 10mm
% - We use Calibratie 1, mid first and then left/right
% save stereoParamsML stereoParamsML 
% save estimationErrorsML estimationErrorsML
% save stereoParamsMR stereoParamsMR
% save estimationErrorsMR estimationErrorsMR

% Load all the parameters 
load 'stereoParamsML.mat'
load 'estimationErrorsML.mat'
load 'stereoParamsMR.mat'
load 'estimationErrorsMR.mat'


%% Task 2: Remove background
% Using kmean to clustering background and shirt, and remove these clusters
imleft=BackgroundRemoveKmean(imleft);
imright=BackgroundRemoveKmean(imright);
immid=BackgroundRemoveKmean(immid);

% Visualization
figure(1)
imshow(imleft)
print('-r300', '-dpng','Task1-BGremoveL');
imshow(immid)
print('-r300', '-dpng','Task1-BGremoveM');
imshow(imright)
print('-r300', '-dpng','Task1-BGremoveR');


%% Task 3: Normalizing color 
[row,col,channels]=size(immid);

%reshape for normalization
imleft_reshape=reshape(imleft,row*col,channels);
imright_reshape=reshape(imright,row*col,channels);
immid_reshape=reshape(immid,row*col,3);

% Normalization for each channel
range_min=min(immid_reshape);
range_max=max(immid_reshape);
imleft_norm=zeros(row*col,channels);
imright_norm=zeros(row*col,channels);
for i=1:channels
    imleft_norm(:,i)=normalize(imleft_reshape(:,i),'range',[range_min(i) range_max(i)]);
    imright_norm(:,i)=normalize(imright_reshape(:,i),'range',[range_min(i) range_max(i)]);
end

% Rebuild the picture
imleft_n=reshape(imleft_norm,[row,col,channels]);
imright_n=reshape(imright_norm,[row,col,channels]);

% Visualize 
figure (2)
imshow(rgb2gray(imleft))
print('-r300', '-dpng','Task2-LeftBWold');
imshow(rgb2gray(immid))
subtitle('Mid')
print('-r300', '-dpng','Task2-Mid');
imshow(rgb2gray(imleft_n))
print('-r300', '-dpng','Task2-LeftBWnew');

figure (2)
imshow(rgb2gray(imright))
print('-r300', '-dpng','Task2-RightBWold');
imshow(rgb2gray(imright_n))
print('-r300', '-dpng','Task2-RightBWnew');

imleft=imleft_n;
imright=imright_n;


%% Task 4: Pre-processing - Undistort the image and stereo rectification
% Undistort the image - already included in stereo rectification
% Stereo rectification of two images
[rect_immidL,rect_imleft]=rectifyStereoImages(immid,imleft,stereoParamsML,'OutputView','full');
[rect_immidR,rect_imright]=rectifyStereoImages(immid,imright,stereoParamsMR,'OutputView','full');

% Visualization - Check if top of the head, and the face features are on same horizontal line 
[mean_angles,rms_angles] = ut_check_rect(rect_imleft,rect_immidL,['visualize']);
print('-r300', '-dpng','Task4-SuccesfulRecleft');
[mean_angles,rms_angles] = ut_check_rect(rect_imright,rect_immidR,['visualize']);
print('-r300', '-dpng','Task4-SuccesfulRecright');


%% Task 5: Stereo matching and Disparity map
% Disparity map parameters
bs = 15;        %defauld bs=15
cTH = 0.7;      %default 0.5
uTH = 0;       %default 15
dTH = [];       %default []

%Convert RGB image gray-level image for disparity map
gray_immidL = rgb2gray(rect_immidL);
gray_imleft = rgb2gray(rect_imleft);
gray_imright = rgb2gray(rect_imright);
gray_immidR = rgb2gray(rect_immidR);

% %Image smoothing
% h=fspecial('gaussian',5,1);
% gray_immidL = imfilter(gray_immidL,h);
% gray_imleft = imfilter(gray_imleft,h);
% gray_imright = imfilter(gray_imright,h);
% gray_immidR= imfilter(gray_immidR,h);

% ------------------------------ LEFT to MID --------------------------------
% - Finding the disparity range
% cpselect(rect_imleft,rect_immidL) % choose a point closest (nose) and choose a point furtherst (ear)
% save Left Left
% save MidL MidL
% [352, 416]
load Left; load MidL
DiffL=MidL-Left; maxDisL=max(DiffL(:,1)); minDisL=min(DiffL(:,1));
disparityRangeL = [ceil(minDisL/16)*16 floor(maxDisL/16)*16]; % within the maxDis and minDis and dividable by 16
%disparityRangeL=[352 416];

% - Find the disparity map
disparityMapL = disparitySGM(gray_immidL,gray_imleft,'DisparityRange',disparityRangeL, 'UniquenessThreshold',uTH);

% - Refine disparity map

% Smoothen
h=fspecial('gaussian', [10 10], 2);
disparityMapL = imfilter(disparityMapL,h);

% Remove background
unreliableL=Unreliable(disparityMapL,rect_immidL);
disparityMapL(unreliableL)=0;

% Interpolation
disparityMapL = interpolationpic(disparityMapL);


% disparityMapL = abs(disparityMapL); % Take absolute value to inverse the depth\
% disparityRangeL = flip(abs(disparityRangeL));

% Visualization
figure; imshow(disparityMapL, disparityRangeL)
title('Disparity Map Left Side');colormap jet; colorbar;
print('-r300', '-dpng','Task5-DisparityLeft');

% ------------------------------ RIGHT to MID --------------------------------
% - Finding the disparity range
%cpselect(rect_imright,rect_immidR) % choose a point closest (nose) and choose a point furtherst (ear)
%save Right Right
%save MidR MidR
load Right; load MidR
DiffR=MidR - Right; maxDisR=max(DiffR(:,1)); minDisR=min(DiffR(:,1));
disparityRangeR = [ceil(minDisR/16)*16 floor(maxDisR/16)*16]; % within the maxDis and minDis and dividable by 8

% - Find the disparity map
disparityMapR = disparitySGM(gray_immidR,gray_imright,'DisparityRange',disparityRangeR, 'UniquenessThreshold',uTH);

% - Refine disparity map

% Smoothen
h=fspecial('gaussian', [10 10], 2);
disparityMapR = imfilter(disparityMapR,h);

%Delete unreliable backgrounds and also any values with NaN
unreliableR=Unreliable(disparityMapR,rect_immidR);
disparityMapR(unreliableR)=0;

% Interpolation
disparityMapL = interpolationpic(disparityMapL);

% Test for point cloud
%disparityMapR(disparityMapR == 0) = NaN;

% Visualization
figure; imshow(disparityMapR,disparityRangeR); 
title("Disparity Map Right Side"); colormap jet; colorbar;
print('-r300', '-dpng','Task5-DisparityRight');

%% Task 6: Create point cloud and mesh

% ------------------------------ LEFT --------------------------------
points3DL = reconstructScene(disparityMapL, stereoParamsML);

% - Visualization
figure
pcshow(points3DL); xlabel('X'); ylabel('Y'); zlabel('Z')
title('Point Cloud Left Side')

% Denoise and Remove noise 
points3DL_cloud = pointCloud(points3DL, 'Color',rect_immidL);
points3DL_denoise=pcdenoise(points3DL_cloud);
points3DL_clean = removeInvalidPoints(points3DL_denoise);

figure()
pcshow(points3DL_clean);
title('Clean Point Cloud Left Side')
% ------------------------------ RIGHT --------------------------------
points3DR = reconstructScene(disparityMapR, stereoParamsMR);

% - Visualization
figure
%pcshow(-points3DR); xlabel('X'); ylabel('Y'); zlabel('Z');
pcshow(points3DR); xlabel('X'); ylabel('Y'); zlabel('Z');
title('Point Cloud Right Side')

% Denoise and Remove noise 
%points3DR_cloud = pointCloud(-points3DR, 'Color',rect_immidR);
points3DR_cloud = pointCloud(points3DR, 'Color',rect_immidR);
points3DR_denoise=pcdenoise(points3DR_cloud);
points3DR_clean = removeInvalidPoints(points3DR_denoise);

figure()
pcshow(points3DR_clean);
title('Cleaner Point Cloud Right Side')


%% Taks 7: Create a Mesh
%mesh_create_func(rect_immidL,disparityMapL,-points3DL,unreliableL)
mesh_create_func(rect_immidL,disparityMapL,points3DL,unreliableL)
title("Left Side Mesh")

mesh_create_func(rect_immidR,disparityMapR,points3DR,unreliableR)
title("Right Side Mesh")


%% Task 8: Combine the Pointclouds

% Subsampling
scale= 1;
points3DL_cloud_down = pcdownsample(points3DL_clean,'random',scale);
points3DR_cloud_down = pcdownsample(points3DR_clean,'random',scale);

% Rotation Matrices
LM = stereoParamsML.RotationOfCamera2;
RM = stereoParamsMR.RotationOfCamera2;

%Initial Transformation
tformI =  affine3d();
tformI.T(1:3,1:3) = RM/LM; %RR*inv(RL);

% Transform the point clouds such that they overlay each other
[tform,movingReg,rmse] = pcregrigid(points3DL_cloud_down,points3DR_cloud_down,'MaxIterations',10, 'InitialTransform', tformI);

%Visiluze point clouds
figure();pcshow(points3DL_cloud_down);
figure();pcshow(points3DR_cloud_down);

% Visualize Transformation
ptCloudAligned = pctransform(points3DL_cloud_down,tform);
pcshow(ptCloudAligned)

% Merge the point clouds 
ptCloudOut = pcmerge(movingReg,points3DR_cloud_down,1);
figure();
pcshow(ptCloudOut);
title("Merged Point Cloud")


%% Task 9: Change to mesh
res=10;
[TM,tri,a]=MESHfinal(ptCloudOut,res);


%% Task 10: Accuracy estimation
th_angle = pi/6;
not_accept=find(min(a,[],2)<th_angle);
acc = 1-length(not_accept)/length(a);

