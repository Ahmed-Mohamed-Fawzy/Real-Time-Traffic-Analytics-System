%% Setup and Initialization
clear all; close all; clc;

% Parameters for processing
gaussianKernelSize = 5;
gaussianSigma = 1.5;
frameBufferSize = 5;          % For vibration removal
detectionThreshold = 0.25;    % Threshold for vehicle detection
minBlobArea = 1000;            % Minimum blob area to be considered a vehicle
maxBlobArea = 8000;          % Maximum blob area to be considered a vehicle
roadROI = [150, 50, 480, 380]; % [x, y, width, height] - Adjust based on your video
countingLineX = 580;          % X-position where vehicles are counted (MODIFIED: changed from countingLineY to countingLineX)

% Open video input and setup video output
try
    % Open input video
    videoReader = VideoReader('top_view_cuted_resized.mp4');
    frameRate = videoReader.FrameRate;
    
    % Setup output video
    outputPath = 'traffic_analysis_output';
    videoWriter = VideoWriter(outputPath, 'MPEG-4');
    videoWriter.FrameRate = frameRate;
    open(videoWriter);
    
    fprintf('Processing video: %s\n', videoReader.Name);
    fprintf('Output will be saved to: %s.mp4\n', outputPath);
    
    % Create figure for visualization
    hFig = figure('Position', [100, 100, videoReader.Width, videoReader.Height*2], 'Name', 'Traffic Analytics');
    
    demoMode = false;
catch
    % If no video file, create a demo visualization
    fprintf('No video file found. Running in demo mode.\n');
    demoMode = true;
    frameWidth = 640;
    frameHeight = 480;
    frameRate = 30;
    
    % Create a figure to visualize the demo
    hFig = figure('Position', [100, 100, frameWidth, frameHeight*2], 'Name', 'Traffic Analytics Demo');
end

% Initialize variables
vehicleCount = 0;
frameBuffer = [];
backgroundModel = [];
trackedVehicles = [];
nextVehicleID = 1;
frameCount = 0;

% Setup for speed estimation
distanceCalibrationFactor = 0.1; % meters per pixel (needs calibration for real system)

%% Main Processing Loop
if ~exist('demoMode', 'var') || ~demoMode
    % Process the entire video file
    fprintf('Starting video processing...\n');
    progressInterval = 10; % Show progress every 10 frames
    
    while hasFrame(videoReader)
        % Read frame from input video
        currentFrame = readFrame(videoReader);
        frameCount = frameCount + 1;
        
        % Process the frame
        % Step 1: Pre-processing with Gaussian smoothing (Spatial domain)
        if size(currentFrame, 3) == 3
            smoothedFrame = imgaussfilt(rgb2gray(currentFrame), gaussianSigma, 'FilterSize', gaussianKernelSize);
        else
            smoothedFrame = imgaussfilt(currentFrame, gaussianSigma, 'FilterSize', gaussianKernelSize);
        end
        
        % Step 2: FFT-based vibration removal (Frequency domain)
        stabilizedFrame = applyVibrationRemoval(smoothedFrame, frameBuffer, frameBufferSize);
        
        % Update frame buffer for future stabilization
        frameBuffer = updateFrameBuffer(frameBuffer, smoothedFrame, frameBufferSize);
        
        % Step 3: Background subtraction with adaptive median filtering (Hybrid approach)
        [foregroundMask, backgroundModel] = adaptiveMedianBackgroundSubtraction(stabilizedFrame, backgroundModel, frameCount);
        
        % Step 4: Vehicle detection and tracking
        [blobStats, labeledImage] = detectVehicles(foregroundMask, minBlobArea, maxBlobArea, roadROI);
        
        % Step 5: Update vehicle tracking
        [trackedVehicles, vehicleCount, nextVehicleID] = trackVehicles(blobStats, trackedVehicles, vehicleCount, nextVehicleID, frameCount);
        
        % Step 6: Estimate vehicle speeds
        trackedVehicles = estimateVehicleSpeeds(trackedVehicles, distanceCalibrationFactor, frameRate);
        
        % Visualization
        visualizeResults(currentFrame, foregroundMask, labeledImage, trackedVehicles, vehicleCount, roadROI);
        
        % Capture the current figure for video output
        frame = getframe(hFig);
        writeVideo(videoWriter, frame.cdata);
        
        % Display progress
        if mod(frameCount, progressInterval) == 0
            fprintf('Processed %d frames, vehicle count: %d\n', frameCount, vehicleCount);
        end
    end
else
    % Process demo mode if needed
    maxFrames = 100; % Adjust as needed for demo
    for frameIdx = 1:maxFrames
        % Generate a demo frame with simulated vehicles
        currentFrame = generateDemoFrame(frameWidth, frameHeight, frameIdx);
        frameCount = frameCount + 1;
        
        % Process the frame (same processing as video mode)
        % Step 1: Pre-processing with Gaussian smoothing (Spatial domain)
        smoothedFrame = imgaussfilt(rgb2gray(currentFrame), gaussianSigma, 'FilterSize', gaussianKernelSize);
        
        % Step 2: FFT-based vibration removal (Frequency domain)
        stabilizedFrame = applyVibrationRemoval(smoothedFrame, frameBuffer, frameBufferSize);
        
        % Update frame buffer for future stabilization
        frameBuffer = updateFrameBuffer(frameBuffer, smoothedFrame, frameBufferSize);
        
        % Step 3: Background subtraction with adaptive median filtering (Hybrid approach)
        [foregroundMask, backgroundModel] = adaptiveMedianBackgroundSubtraction(stabilizedFrame, backgroundModel, frameCount);
        
        % Step 4: Vehicle detection and tracking
        [blobStats, labeledImage] = detectVehicles(foregroundMask, minBlobArea, maxBlobArea, roadROI);
        
        % Step 5: Update vehicle tracking
        [trackedVehicles, vehicleCount, nextVehicleID] = trackVehicles(blobStats, trackedVehicles, vehicleCount, nextVehicleID, frameCount);
        
        % Step 6: Estimate vehicle speeds
        trackedVehicles = estimateVehicleSpeeds(trackedVehicles, distanceCalibrationFactor, frameRate);
        
        % Visualization
        visualizeResults(currentFrame, foregroundMask, labeledImage, trackedVehicles, vehicleCount, roadROI);
        
        % Pause to simulate real-time processing
        pause(0.05);
    end
end

% Close all resources
if exist('videoWriter', 'var')
    % Fix for isOpen error - use proper object checking
    if isobject(videoWriter)
        close(videoWriter);
        fprintf('Output video saved to: %s.mp4\n', outputPath);
    end
end

fprintf('Processing complete. Total vehicles counted: %d\n', vehicleCount);

%% Helper Functions

function stabilizedFrame = applyVibrationRemoval(currentFrame, frameBuffer, bufferSize)
    % FFT-based vibration removal
    % In a real implementation, this would use phase correlation or similar techniques
    
    % If not enough frames in buffer yet, return original frame
    if length(frameBuffer) < bufferSize
        stabilizedFrame = currentFrame;
        return;
    end
    
    % Convert to frequency domain
    F = fft2(double(currentFrame));
    
    % Apply low-pass filter to reduce high-frequency noise (vibrations)
    [rows, cols] = size(currentFrame);
    [u, v] = meshgrid(1:cols, 1:rows);
    center_r = floor(rows/2) + 1;
    center_c = floor(cols/2) + 1;
    u = u - center_c;
    v = v - center_r;
    
    % Create a low-pass filter in the frequency domain
    radius = min(rows, cols) / 8;  % Cutoff radius
    lpFilter = exp(-(u.^2 + v.^2) / (2*radius^2));
    
    % Apply filter in frequency domain
    F_filtered = F .* fftshift(lpFilter);
    
    % Convert back to spatial domain
    stabilizedFrame = uint8(real(ifft2(F_filtered)));
end

function updatedBuffer = updateFrameBuffer(buffer, newFrame, maxSize)
    % Add new frame to buffer and maintain max size
    buffer = [buffer, {newFrame}];
    if length(buffer) > maxSize
        buffer = buffer(2:end);
    end
    updatedBuffer = buffer;
end

function [foregroundMask, updatedModel] = adaptiveMedianBackgroundSubtraction(frame, model, frameCount)
    % Implement adaptive median-based background subtraction
    
    % Initialize background model with first frame
    if isempty(model) || frameCount <= 3
        updatedModel = frame;
        foregroundMask = false(size(frame));
        return;
    end
    
    % Update model (adaptive part)
    alpha = 0.05; % Learning rate
    updatedModel = uint8((1 - alpha) * double(model) + alpha * double(frame));
    
    % Calculate absolute difference between current frame and background model
    frameDiff = abs(double(frame) - double(model));
    
    % Apply adaptive threshold
    threshold = 26; % Can be adjusted or made adaptive
    foregroundMask = frameDiff > threshold;
    
    % Apply morphological operations to clean up the mask
    foregroundMask = imopen(foregroundMask, strel('disk', 2));
    foregroundMask = imclose(foregroundMask, strel('disk', 10));
    foregroundMask = imfill(foregroundMask, 'holes');
end

function [blobStats, labeledImage] = detectVehicles(foregroundMask, minArea, maxArea, roi)
    % Get dimensions of the foreground mask
    [height, width] = size(foregroundMask);
    
    % Make sure ROI is within image bounds
    roi(1) = max(1, roi(1));
    roi(2) = max(1, roi(2));
    roi(3) = min(width - roi(1) + 1, roi(3));
    roi(4) = min(height - roi(2) + 1, roi(4));
    
    % Extract region of interest
    roiMask = false(size(foregroundMask));
    if roi(3) > 0 && roi(4) > 0 && roi(1) + roi(3) - 1 <= width && roi(2) + roi(4) - 1 <= height
        roiMask(roi(2):roi(2)+roi(4)-1, roi(1):roi(1)+roi(3)-1) = true;
        foregroundMask = foregroundMask & roiMask;
    else
        % If ROI is invalid, just use the full mask
        fprintf('Warning: Invalid ROI dimensions. Using full frame.\n');
    end
    
    % Connected component analysis
    [labeledImage, numComponents] = bwlabel(foregroundMask, 8);
    
    % Extract blob statistics
    stats = regionprops(labeledImage, 'Area', 'Centroid', 'BoundingBox');
    
    % Filter blobs by area
    if isempty(stats)
        blobStats = [];
        labeledImage = zeros(size(foregroundMask));
        return;
    end
    
    validIndices = find([stats.Area] >= minArea & [stats.Area] <= maxArea);
    
    % Update labeled image to only include valid blobs
    newLabeledImage = zeros(size(labeledImage));
    for i = 1:length(validIndices)
        newLabeledImage(labeledImage == validIndices(i)) = i;
    end
    
    blobStats = stats(validIndices);
    labeledImage = newLabeledImage;
end

function [updatedTrackedVehicles, newCount, nextID] = trackVehicles(blobStats, trackedVehicles, currentCount, currentNextID, frameNumber)
    % Simple tracking algorithm based on centroid proximity
    
    % Initialize output
    updatedTrackedVehicles = trackedVehicles;
    newCount = currentCount;
    nextID = currentNextID;
    
    % Get countingLineX from parent workspace (MODIFIED: changed from countingLineY to countingLineX)
    countingLineX = evalin('caller', 'countingLineX');
    
    % If no tracked vehicles yet, initialize with detected blobs
    if isempty(trackedVehicles)
        for i = 1:length(blobStats)
            newVehicle.id = nextID;
            newVehicle.centroid = blobStats(i).Centroid;
            newVehicle.boundingBox = blobStats(i).BoundingBox;
            newVehicle.firstSeen = frameNumber;
            newVehicle.lastSeen = frameNumber;
            newVehicle.positions = [blobStats(i).Centroid];
            newVehicle.counted = false;
            newVehicle.speed = 0;
            
            updatedTrackedVehicles = [updatedTrackedVehicles, newVehicle];
            nextID = nextID + 1;
        end
        return;
    end
    
    % Mark all tracked vehicles as not updated in this frame
    for i = 1:length(updatedTrackedVehicles)
        updatedTrackedVehicles(i).updated = false;
    end
    
    % For each detected blob, find closest tracked vehicle
    for i = 1:length(blobStats)
        blob = blobStats(i);
        minDist = inf;
        bestMatch = 0;
        
        for j = 1:length(updatedTrackedVehicles)
            % Only match with recently seen vehicles (within last 5 frames)
            if frameNumber - updatedTrackedVehicles(j).lastSeen <= 5
                dist = norm(blob.Centroid - updatedTrackedVehicles(j).centroid);
                if dist < minDist && dist < 50  % Maximum matching distance
                    minDist = dist;
                    bestMatch = j;
                end
            end
        end
        
        if bestMatch > 0
            % Update existing track
            updatedTrackedVehicles(bestMatch).centroid = blob.Centroid;
            updatedTrackedVehicles(bestMatch).boundingBox = blob.BoundingBox;
            updatedTrackedVehicles(bestMatch).lastSeen = frameNumber;
            updatedTrackedVehicles(bestMatch).positions = [updatedTrackedVehicles(bestMatch).positions; blob.Centroid];
            updatedTrackedVehicles(bestMatch).updated = true;
            
            % Check if vehicle crossed counting line (MODIFIED: changed Y comparison to X comparison)
            if ~updatedTrackedVehicles(bestMatch).counted && ...
               updatedTrackedVehicles(bestMatch).centroid(1) > countingLineX && ...
               length(updatedTrackedVehicles(bestMatch).positions) > 5 && ...
               size(updatedTrackedVehicles(bestMatch).positions, 1) >= 2 && ... 
               updatedTrackedVehicles(bestMatch).positions(end-1, 1) <= countingLineX
                updatedTrackedVehicles(bestMatch).counted = true;
                newCount = newCount + 1;
            end
        else
            % Create new track
            newVehicle.id = nextID;
            newVehicle.centroid = blob.Centroid;
            newVehicle.boundingBox = blob.BoundingBox;
            newVehicle.firstSeen = frameNumber;
            newVehicle.lastSeen = frameNumber;
            newVehicle.positions = [blob.Centroid];
            newVehicle.counted = false;
            newVehicle.speed = 0;
            newVehicle.updated = true;
            
            updatedTrackedVehicles = [updatedTrackedVehicles, newVehicle];
            nextID = nextID + 1;
        end
    end
    
    % Remove vehicles that haven't been seen recently
    activeVehicles = [];
    for i = 1:length(updatedTrackedVehicles)
        if frameNumber - updatedTrackedVehicles(i).lastSeen <= 10
            activeVehicles = [activeVehicles, updatedTrackedVehicles(i)];
        end
    end
    updatedTrackedVehicles = activeVehicles;
end

function updatedVehicles = estimateVehicleSpeeds(vehicles, distanceFactor, frameRate)
    % Estimate vehicle speeds based on position history
    updatedVehicles = vehicles;
    
    for i = 1:length(vehicles)
        % Need at least 10 positions for reliable speed estimation
        if size(vehicles(i).positions, 1) >= 10
            % Get last 10 positions
            recentPositions = vehicles(i).positions(end-9:end, :);
            
            % Calculate total distance traveled in pixels
            totalDist = 0;
            for j = 2:size(recentPositions, 1)
                totalDist = totalDist + norm(recentPositions(j,:) - recentPositions(j-1,:));
            end
            
            % Convert to meters using calibration factor
            distanceMeters = totalDist * distanceFactor;
            
            % Calculate time elapsed (in seconds)
            timeElapsed = 9 / frameRate;  % 9 intervals between 10 positions
            
            % Calculate speed in meters per second
            speedMPS = distanceMeters / timeElapsed;
            
            % Convert to km/h
            speedKMH = speedMPS * 3.6;
            
            updatedVehicles(i).speed = speedKMH;
        end
    end
end

function visualizeResults(originalFrame, foregroundMask, labeledImage, trackedVehicles, vehicleCount, roi)
    % Create visualization of results
    
    % Display original frame with annotations
    subplot(2, 1, 1);
    imshow(originalFrame);
    hold on;
    
    % Draw ROI
    rectangle('Position', roi, 'EdgeColor', 'y', 'LineWidth', 2, 'LineStyle', '--');
    
    % Draw counting line (MODIFIED: changed from horizontal to vertical line)
    countingLineX = evalin('caller', 'countingLineX');
    line([countingLineX, countingLineX], [1, size(originalFrame, 1)], 'Color', 'r', 'LineWidth', 2, 'LineStyle', '-');
    text(countingLineX + 10, 20, 'Counting Line', 'Color', 'r', 'FontWeight', 'bold', 'BackgroundColor', [1 1 1 0.7]);
    
    % Draw bounding boxes and IDs for tracked vehicles
    for i = 1:length(trackedVehicles)
        vehicle = trackedVehicles(i);
        % Access frameCount from the parent workspace
        frameCount = evalin('caller', 'frameCount');
        if vehicle.lastSeen == frameCount
            rectangle('Position', vehicle.boundingBox, 'EdgeColor', 'g', 'LineWidth', 2);
            text(vehicle.centroid(1), vehicle.centroid(2), sprintf('ID: %d\n%.1f km/h', ...
                 vehicle.id, vehicle.speed), 'Color', 'r', 'FontWeight', 'bold', ...
                 'BackgroundColor', [1 1 1 0.7]);
                 
            % Draw trajectory
            if size(vehicle.positions, 1) > 1
                plot(vehicle.positions(:,1), vehicle.positions(:,2), 'r-', 'LineWidth', 2);
            end
        end
    end
    
    % Add vehicle count
    text(10, 30, sprintf('Vehicle Count: %d', vehicleCount), 'Color', 'r', 'FontWeight', 'bold', ...
         'FontSize', 14, 'BackgroundColor', [1 1 1 0.7]);
    
    % Add timestamp
    % Access frameCount and frameRate from the parent workspace
    frameCount = evalin('caller', 'frameCount');
    if evalin('caller', 'exist(''videoReader'', ''var'')')
        frameRate = evalin('caller', 'frameRate');
        currentTime = frameCount / frameRate;
        text(10, size(originalFrame, 1) - 30, sprintf('Time: %.2f s', currentTime), 'Color', 'r', 'FontWeight', 'bold', ...
                'FontSize', 12, 'BackgroundColor', [1 1 1 0.7]);
    end
    hold off;
    
    % Display processing results
    subplot(2, 1, 2);
    
    % Create a colored visualization of the labeled image
    coloredLabels = label2rgb(labeledImage, 'jet', 'k', 'shuffle');
    
    % Combine the mask and labels
    processedView = imfuse(foregroundMask, coloredLabels, 'blend');
    imshow(processedView);
    title('Foreground Mask & Vehicle Detection');
    
    % Give MATLAB time to update the display properly
    drawnow;
end

function demoFrame = generateDemoFrame(width, height, frameNumber)
    % Generate a demo frame with simulated moving vehicles
    % Create a road background
    demoFrame = uint8(ones(height, width, 3) * 100);  % Gray background
    
    % Draw road
    roadTop = height/3;
    roadBottom = 2*height/3;
    for i = 1:3
        demoFrame(round(roadTop):round(roadBottom), :, i) = uint8(ones(round(roadBottom-roadTop)+1, width) * 50);
    end
    
    % Draw lane markings
    laneWidth = 20;
    for i = 1:10:width
        if mod(i, 100) < 60  % Dashed lines
            startX = i;
            endX = min(i+40, width);
            midLine = round((roadTop+roadBottom)/2);
            for c = 1:3
                if c == 3
                    val = 50; % Yellow
                else
                    val = 200; % Yellowish-white
                end
                demoFrame(midLine-1:midLine+1, startX:endX, c) = uint8(ones(3, endX-startX+1) * val);
            end
        end
    end
    
    % Add vehicles
    numVehicles = 5;
    for i = 1:numVehicles
        % Vehicle parameters
        vehicleWidth = 40 + randi(20);
        vehicleHeight = 25 + randi(10);
        vehicleSpeed = 5 + randi(5);
        vehicleLane = (i <= numVehicles/2);  % Top or bottom lane
        
        % Vehicle position
        xPos = mod((i*100) + frameNumber * vehicleSpeed, width + vehicleWidth) - vehicleWidth;
        if vehicleLane
            yPos = round((roadTop+roadBottom)/2) - vehicleHeight - 5;
        else
            yPos = round((roadTop+roadBottom)/2) + 5;
        end
        
        % Draw vehicle (if in frame)
        if xPos < width && xPos + vehicleWidth > 0
            startX = max(1, xPos);
            endX = min(width, xPos + vehicleWidth);
            startY = max(1, yPos);
            endY = min(height, yPos + vehicleHeight);
            
            % Random vehicle color
            vehicleColor = randi(200, [1, 1, 3]) + 55;
            
            for c = 1:3
                demoFrame(startY:endY, startX:endX, c) = uint8(ones(endY-startY+1, endX-startX+1) * vehicleColor(c));
            end
        end
    end
end