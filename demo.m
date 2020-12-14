close all; clear; clc
%% Basic Parameters
ReadPath = './examples';
name = 'example01.png';  % example01 - example05

% RealSense parameters, used for computing disparity
Baseline = 55.0871;
Focallength = 1367.6650;
%% Read Data and Preprocess
rgb = imread(fullfile(ReadPath, 'rgb', name));
distance = double(imread(fullfile(ReadPath, 'depth_u16', name)));
distance(distance > 10000) = 0;     % do not consider the pixels with distances over 10m (RealSense depth range)
disparity = Focallength * Baseline ./ distance;

rgb = preprocess_input(rgb);
disparity = preprocess_input(disparity);
disparity(isinf(disparity)) = 0;
disparity = round(disparity);

disp('Successfully read data.');
%% Compute V-disparity
% v-disparity
m = size(disparity, 1);
n = max(max(disparity)) + 1;
vdis = zeros(m, n);
for i = 1 : m
    for j = 1 : n
        vdis(i, j) = length(find(disparity(i, :) == (j - 1)));
    end
end

% steerable filter
theta = [0, 45, 90];
vdisSteerable = zeros(size(vdis, 1), size(vdis, 2), 3); % use steerable filter in 3 directions
for i = 1 : length(theta)
    vdisSteerable(:, :, i) = steerGaussFilterOrder2(vdis, theta(i), 3);
end

% select the pixels that have much difference between 3 directions
vdisSteerableDiff = zeros(size(vdis));
for i = 1 : size(vdisSteerable, 1)
    for j = 1 : size(vdisSteerable, 2)
        vdisSteerableDiff(i, j) = max(vdisSteerable(i, j, :)) - min(vdisSteerable(i, j, :));
    end
end
vdisFilter = zeros(size(vdis));
vdisFilterThresh = 30;
vdisFilter(vdisSteerableDiff > vdisFilterThresh) = 1;

disp('Successfully compute v-disparity.');
%% Drivable Area Segmentation
% Hough Transform
[line, status] = HoughTransform(vdisFilter);
if(status == 0)
    disp('Hough transform failed.');
    return
end
point1 = line.point1;
point2 = line.point2;

% perform drivable area segmentation based on Hough Transform
drivableInitial = zeros(size(disparity));
drivableInitialThresh = 3;
for i = point1(2) : point2(2)
    d = (point2(1) - point1(1)) / (point2(2) - point1(2)) * i + (point1(1) * point2(2) - point2(1) * point1(2)) / (point2(2) - point1(2));
    for j = 1 : size(drivableInitial, 2)
        if(disparity(i, j) > d - drivableInitialThresh && disparity(i, j) < d + drivableInitialThresh)
            drivableInitial(i, j) = 1;
        end
    end
end
drivableFinal = medfilt2(drivableInitial, [5, 5]);

disp('Successfully perform drivable area segmentation.');
%% Depth Anomaly Map Generation
drivableBW = drivableFinal;     % used for boundary detection
drivableBW(1, :) = 0; drivableBW(end, :) = 0;
drivableBW(:, 1) = 0; drivableBW(:, end) = 0;
[B, L, N] = bwboundaries(~drivableBW);

% select the holes with a specific area range
depthAnom = zeros(size(drivableBW));
for i = 1 : N
    temp = zeros(size(drivableBW));
    boundary = B{i};
    for j = 1 : size(boundary, 1)
        temp(boundary(j, 1), boundary(j, 2)) = 1;
    end
    area = bwarea(temp);
    if area > 25 && area < 500
        for j = 1 : size(boundary, 1)
            depthAnom(boundary(j, 1), boundary(j, 2)) = 1;
        end
    end
end
depthAnom = imfill(depthAnom, 'holes');
drivablePlusDepthAnom = depthAnom | drivableFinal;

disp('Successfully generate the depth anomaly map.');
%% RGB Anomaly Map Generation
lab = rgb2lab(rgb);
labFilter = gaussFilter(lab, 12);
rgbAnom = sum((labFilter - lab).^2, 3);
rgbAnom(drivablePlusDepthAnom == 0) = 0;
rgbAnom = mapminmax(rgbAnom, 0, 1);

disp('Successfully generate the RGB anomaly map.');
%% Road Anomaly Segmentation
anomaly = 0.5 * depthAnom + 0.5 * rgbAnom;
anomalyFinal = zeros(size(anomaly));
anomalyThresh = 0.5;
anomalyFinal(anomaly >= anomalyThresh) = 1;
anomalyFinal = medfilt2(anomalyFinal, [5, 5]);

disp('Successfully perform road anomaly segmentation.');
%% Save Generated Label and Visulization
label  = uint8(zeros(size(disparity)));
label(drivablePlusDepthAnom == 1) = 1;
label(anomalyFinal == 1) = 2;
vis = visualization(rgb, label);

mkdir(fullfile(ReadPath, 'output'));
imwrite(label, fullfile(ReadPath, 'output', [name(1:end-4), '_SSLG.png']));
imwrite(vis, fullfile(ReadPath, 'output', [name(1:end-4), '_SSLG_vis.png']));
figure, imshow(vis);

disp('Successfully save the generated label.');
%% Functions
function output = preprocess_input(input)
    % crop invalid boundaries, and resize the input image
    % Input:
    %   input: input image
    % Output:
    %   output: output image
    
    output = imresize(input(1 : 690, 181 : 1100, :), [480, 640]);
end

function J = steerGaussFilterOrder2(I, theta, sigma)
    % the steerable filter with the second derivatives of Gaussian
    % Input:
    %   1. I: input image
    %   2. theta: the orientation
    %   3. sigma: standard deviation of the Gaussian template
    % Output:
    %   J: The response of derivative in theta direction

    theta = -theta * (pi / 180);

    % determine necessary filter support (for Gaussian)
    Wx = floor((8 / 2) * sigma); 
    if Wx < 1
      Wx = 1;
    end
    x = [-Wx : Wx];

    [xx, yy] = meshgrid(x, x);

    g0 = exp(-(xx.^2 + yy.^2) / (2 * sigma^2)) / (sigma * sqrt(2 * pi));
    G2a = -g0 / sigma^2 + g0 .* xx.^2 / sigma^4;
    G2b =  g0 .* xx .* yy / sigma^4;
    G2c = -g0 / sigma^2 + g0 .* yy.^2 / sigma^4;

    % compute image gradients (using separability)
    I2a = imfilter(I, G2a, 'same', 'replicate');
    I2b = imfilter(I, G2b, 'same', 'replicate');
    I2c = imfilter(I, G2c, 'same', 'replicate');

    % evaluate oriented filter response
    J = (cos(theta))^2 * I2a + sin(theta)^2 * I2c - 2 * cos(theta) * sin(theta) * I2b;
end

function [outputline, status] = HoughTransform(I)
    % perform Hough Transform, and return one dominant line
    % Input:
    %   I: input image
    % Output:
    %   1. outputline: output line
    %   2. status: 1 for success, and 0 for failure

    % exatract image boundary based on Canny
    BW = edge(I, 'canny');
    for i = 1 : size(BW, 1)
        flag = 0;
        for j = size(BW, 2) : -1 : 1
            if flag == 0 && BW(i, j) == 0
                continue
            else
                if flag == 0 && BW(i, j) == 1
                    flag = 1;
                    BW(i, j) = 0;
                else
                    if flag == 1 && BW(i, j) == 1
                        BW(i, j) = 0;
                    else
                        break
                    end
                end
            end
        end
    end

    % perform Hough Transform
    [H, T, R] = hough(BW);
    P = houghpeaks(H, 1);
    lines = houghlines(BW, T, R, P);

    % select the longest line
    if length(lines) == 1
        outputline = lines;
    else
        lenarray = zeros(length(lines), 1);
        for k = 1 : length(lines)
            point1 = lines(k).point1;
            point2 = lines(k).point2;
            lenarray(k) = sum((point1 - point2).^2);
        end
        [~, pos] = max(lenarray);
        outputline = lines(pos);
    end

    if(isempty(outputline))
        status = 0;
        outputline = 0;
        return
    end

    % extend the detected line
    [height, width] = size(I);

    point1 = outputline.point1; point2 = outputline.point2;
    temp1 = point1; temp2 = point2;
    leftflag = 1; rightflag = 1;
    interval = 5; index = 3;

    while (leftflag && point1(1) > interval && point1(2) > 1) || (rightflag && point2(1) < width - interval + 1 && point2(2) < height)
        if leftflag && point1(1) > interval && point1(2) > 1
            temp1(1) = point1(1) - interval;
            temp1(2) = round(point1(2) - interval * (point2(2) - point1(2)) / (point2(1) - point1(1)));
            lbound = max(1, temp1(1) - index);
            if temp1(2) > 1 && sum(BW(temp1(2), lbound : temp1(1) + index)) > 0
                temp1(1) = temp1(1) - length((lbound : temp1(1) + index)) + index + find(BW(temp1(2), lbound : temp1(1) + index) > 0, 1);
                point1 = temp1;
            else
                leftflag = 0;
            end
        end

        if rightflag && point2(1) < width - interval + 1 && point2(2) < height
            temp2(1) = point2(1) + interval;
            temp2(2) = round(point2(2) + interval * (point2(2) - point1(2)) / (point2(1) - point1(1)));
            rbound = min(temp2(1) + index, width);
            if temp2(2) < height && sum(BW(temp2(2), temp2(1) - index : rbound)) > 0
                temp2(1) = temp2(1) - index - 1 + find(BW(temp2(2), temp2(1) - index : rbound) > 0, 1);
                point2 = temp2;
            else
                rightflag = 0;
            end
        end
    end
    outputline.point1 = point1;
    outputline.point2 = point2;
    status = 1;
end

function output = gaussFilter(I, sigma)
    % the gaussian filter
    % Input:
    %   1. I: input image
    %   2. sigma: standard deviation of the Gaussian template
    % Output:
    %   output: output image

    output = I;
    sigma = round(min(size(I, 1), size(I, 2)) / sigma);
    ksize = double(3 * sigma);
    window = fspecial('gaussian', [1, ksize], sigma);
    for i = 1 : size(I, 3)
        ret = imfilter(I(:, :, i), window, 'replicate');
        ret = imfilter(ret, window', 'replicate');
        output(:, :, i) = ret;
    end
end

function vis = visualization(rgb, label)
    vis_color = [255, 0, 0;
                0, 255, 0;
                0, 0, 255];
    scale = 0.5;

    vis = rgb;
    for i = 1 : 3
        temp = vis(:, :, i);
        temp(label == 0) = (1 - scale) * temp(label == 0) + scale * vis_color(3, i);
        temp(label == 1) = (1 - scale) * temp(label == 1) + scale * vis_color(2, i);
        temp(label == 2) = (1 - scale) * temp(label == 2) + scale * vis_color(1, i);
        vis(:, :, i) = temp;
    end
end
