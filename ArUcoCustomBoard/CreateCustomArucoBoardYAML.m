% Create a YAML parameter file that defines a custom ArUco board that may have
% markers missing or in non-tabular positions or a 3D board (Z-component
% varies). Each of the ArUco markers have a unique ID and four corners and
% come from a dictonary of markers. I have only worked with DICT_4X4
% dictonary which means the tags are defined by 4x4 arrangement of
% black/white squares.
%
% You will need to list the marker IDs that are present in the board and
% the side length of the markers in metres
% Please stick to the 4x4 dictonary as you have potentially 2^16 tags. The
% more dense dictionaries (E.g. Apriltags, 5X5, 6x6, ...) are harder to
% detect and generally require higher resolution images.
% 
% Author: Jasprabhjit Mehami, 13446277

close all;
clear;

%robotics toolbox for visualising
% run(['rvctools' filesep 'startup_rvc.m']);

%yaml reader package
addpath(genpath(fullfile('..','ext_lib','yamlmatlab-master')));


%% Ask for type of board


listOptions = {
    'line-scan frame camera calibration', 'light source mapping', 'double-sided rig board', 'wool scanning platform', 'test 2x2'
    };

[boardType,tf] = listdlg('ListString',listOptions, 'PromptString', "What type of ArUco board are you using?", 'ListSize', [500, 250], 'SelectionMode', 'single');

if ~tf
    error('no option was selected');
end


%% Creating board parameters

switch boardType
    case 1
        markerSideLen = 0.035;
        sepLen = 0.013;
        numRows = 8;
        numCols = 6;
        numMarkers = 30;
        skipRows = 3:5;
        
        curID = 0;
        ind = 1;
        markerCorners = zeros(numMarkers, 12);
        markerIDs = zeros(1,numMarkers, 'int8');
        
        for i = 1:numRows
            for j = 1:numCols
                
                if ~any(i == skipRows)
                    % bottom-left corner of marker
                    xBL = (j-1)*markerSideLen + (j - 1)*sepLen;
                    yBL = (numRows-i)*markerSideLen + (numRows-i)*sepLen;
                    zBL = 0;
                    
                    curMarkerCorners = GetMarkerCornersXYZ(xBL, yBL, zBL, markerSideLen);
                    markerCorners(ind,:) = curMarkerCorners;
                    markerIDs(ind) = curID;
                    ind = ind + 1;
                end
                
                curID = curID + 1;
            end
        end
        
    case 2
        markerSideLen = 0.035;
        sepLen = 0.013;
        numRows = 8;
        numCols = 6;
        numMarkers = 42;
        skipRows = 4;
        
        curID = 0;
        ind = 1;
        markerCorners = zeros(numMarkers, 12);
        markerIDs = zeros(1,numMarkers, 'int8');
        
        for i = 1:numRows
            for j = 1:numCols
                
                if ~any(i == skipRows)
                    % bottom-left corner of marker
                    xBL = (j-1)*markerSideLen + (j - 1)*sepLen;
                    yBL = (numRows-i)*markerSideLen + (numRows-i)*sepLen;
                    zBL = 0;
                    
                    curMarkerCorners = GetMarkerCornersXYZ(xBL, yBL, zBL, markerSideLen);
                    markerCorners(ind,:) = curMarkerCorners;
                    markerIDs(ind) = curID;
                    ind = ind + 1;
                end
                
                curID = curID + 1;
            end
        end
        
    case 3
        markerSideLen = 0.04;
        sepLen = 0.02;
        numRows = 4;
        numCols = 7;
        numMarkers = 2*numRows*numCols;
        
        curID = 0;
        ind = 1;
        markerCorners = zeros(numMarkers, 12);
        markerIDs = zeros(1,numMarkers, 'int8');
        
        %Front side
        for i = 1:numRows
            for j = 1:numCols
                
                % bottom-left corner of marker
                xBL = (j-1)*markerSideLen + (j - 1)*sepLen;
                yBL = (numRows-i)*markerSideLen + (numRows-i)*sepLen;
                zBL = 0;
                
                curMarkerCorners = GetMarkerCornersXYZ(xBL, yBL, zBL, markerSideLen);
                markerCorners(ind,:) = curMarkerCorners;
                markerIDs(ind) = curID;
                ind = ind + 1;
                curID = curID + 1;
            end
        end
        
        %Back side
        for i = 1:numRows
            for k = 1:numCols
                
                % bottom-left corner of marker
                j = numCols-k+1;
                xBL = (j-1)*markerSideLen + (j - 1)*sepLen;
                yBL = (numRows-i)*markerSideLen + (numRows-i)*sepLen;
                zBL = -0.0066;
                
                curMarkerCorners = GetMarkerCornersXYZ(xBL, yBL, zBL, markerSideLen);
                markerCorners(ind,:) = curMarkerCorners;
                markerIDs(ind) = curID;
                ind = ind + 1;
                curID = curID + 1;
            end
        end
        
    case 5
        markerSideLen = 0.1;
        sepLen = 0.2;
        numRows = 2;
        numCols = 2;
        numMarkers = 4;
        
        curID = 0;
        ind = 1;
        markerCorners = zeros(numMarkers, 12);
        markerIDs = zeros(1,numMarkers, 'int8');
        
        for i = 1:numRows
            for j = 1:numCols
                
                
                % bottom-left corner of marker
                xBL = (j-1)*markerSideLen + (j - 1)*sepLen;
                yBL = (numRows-i)*markerSideLen + (numRows-i)*sepLen;
                zBL = 0;
                
                curMarkerCorners = GetMarkerCornersXYZ(xBL, yBL, zBL, markerSideLen);
                markerCorners(ind,:) = curMarkerCorners;
                markerIDs(ind) = curID;
                ind = ind + 1;
                
                
                curID = curID + 1;
            end
        end
end

%plot aruco board to compare with actual
PlotArucoBoard(markerIDs, markerCorners);

%save YAML file with IDs and consective marker corners
paramStruct.objPoints = round(markerCorners, 5);
paramStruct.ids = markerIDs;
paramStruct.dictionary = '4x4';
yaml.WriteYaml('aruco-board-markers.yaml', paramStruct);





%% Helper functions

function corners = GetMarkerCornersXYZ(xBL, yBL, zBL, sideLen)
% Given the bottom-left corner of a ArUco marker and its side length, this function will return
% an array of all corner XYZ positions. The marker is assumed to be from a
% a flat board, so the Z coordinate is the same for all corners.
%
% OUTPUT:
%       corners - corner coordinate positions where each corner is a [x,y,z]
%           Corners are in sequence [top-left, top-right, bottom-right, bottom-left]

corners = zeros(1,12);

%save top-left corner
x = xBL;
y = yBL + sideLen;
z = zBL;
corners(1:3) = [x,y,z];

%save top-right corner
x = xBL + sideLen;
y = yBL + sideLen;
z = zBL;
corners(4:6) = [x,y,z];

%save bottom-right corner
x = xBL + sideLen;
y = yBL;
z = zBL;
corners(7:9) = [x,y,z];

%save bottom-left corner
x = xBL;
y = yBL;
z = zBL;
corners(10:12) = [x,y,z];
end

function fig = PlotArucoBoard(ids, corners)
% Plots a 3D figure of the ArUco board given marker IDs and corners.

numMarkers = length(ids);

if numMarkers ~= size(corners, 1)
    error("number of marker IDS does not match size of corners matrix")
end

fig = figure('Name', 'ArUco board');
hold on;

%plot square markers and mark ID
for i = 1:numMarkers
    plot3(corners(i,[1:3:12, 1]), corners(i,[2:3:12, 2]), corners(i,[3:3:12, 3]),  'k-');
    text(mean(corners(i,1:3:12)), mean(corners(i,2:3:12)), mean(corners(i,3:3:12)), num2str(ids(i)), 'HorizontalAlignment', 'center');
end

trplot(eye(4), 'frame', 'Pat', 'rviz', 'length', 0.1);

hold off;
axis equal; grid off;
xyzlabel();

end


