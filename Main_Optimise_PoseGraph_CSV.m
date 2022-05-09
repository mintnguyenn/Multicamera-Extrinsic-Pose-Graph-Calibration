% This script will optimise a posegraph of a rigid multi-camera system that
% consists of N cameras that see M Board poses. The nodes and edges are
% defined in separate CSV files. The first node is the origin.
% The first N nodes are the pose of each camera w.r.t to the origin, and
% then the next M nodes are the poses of a detected board w.r.t to the origin.
% The edges are the connections between N camera nodes and the M Board
% nodes.
%
% Note that each camera is not guaranteed to have a edge with each
% board, but a single board must have atleast 2 edges to be useful in the pose graph
%
% Author: Jasprabhjit Mehami
clc;
close all;
clear;

%robotics toolbox
run(fullfile('ext_lib','rvctools', 'startup_rvc.m'));


%% Number of cameras to calibrate

numCam = 4;


%% Read CSV files

fprintf("Reading CSV files ...")

csvDir = 'pose_graph_csv';

edgesCSV = fullfile(csvDir, 'Edges.csv');
nodesCSV = fullfile(csvDir, 'Nodes.csv');

if ~isfile(edgesCSV)
    error("Edges.csv is missing");
end

if ~isfile(nodesCSV)
    error("Nodes.csv is missing");
end

edgesMat = readmatrix(edgesCSV, 'NumHeaderLines', 1);

%remove edges that are were not found. These are edges where the
%transformation is identity matrix (check if qw is close to 1)
goodEdgeInd = edgesMat(:,end) ~= 1;
edgesMat = edgesMat(goodEdgeInd,:);

nodesMat = readmatrix(nodesCSV, 'NumHeaderLines', 1);
goodNodes = sum(isnan(nodesMat), 2) < 1;
nodesMat = nodesMat(goodNodes, :);

numNodes = size(nodesMat, 1);
numEdges = size(edgesMat, 1);

%camera and board ID offsets because the camera IDs don't start from 1
camIDoffset = 6;
boardIDoffset = nodesMat(numCam+1, 1) - nodesMat(numCam, 1) + camIDoffset - 1;

fprintf("DONE\n")


%% Setup pose graph

fprintf("Setting up pose graph ...")

poseGraph = poseGraph3D('MaxNumEdges',numEdges,'MaxNumNodes',numNodes);

%default information matrix (information matrix is inverse of covariance
%and accounts for noise)
inforMatDefault = [1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 1 0 0 1 0 1];

%the reference node
fromNode = 1;

%add nodes to posegraph
for i = 2:numNodes
    curPose = Convert2MatlabPoseOrder(nodesMat(i,2:end));
    addRelativePose(poseGraph, curPose, inforMatDefault, fromNode);
end

figure('Name', 'All nodes')
show(poseGraph, 'IDs', 'nodes');

%add edges to posegraph
for i = 1:numEdges
    curPose = Convert2MatlabPoseOrder(edgesMat(i,3:end));
    camNode = edgesMat(i,1) - camIDoffset;
    boardNode = edgesMat(i,2) - boardIDoffset;
    addRelativePose(poseGraph, curPose, inforMatDefault, camNode, boardNode);
end

fprintf("DONE\n")

%% Perform optimisation

figure('Name', 'All nodes and edges')
show(poseGraph, 'IDs', 'nodes');

fprintf("Optimising pose graph ...")
optmPoseGraph = optimizePoseGraph(poseGraph);
fprintf("DONE\n")


%% Show results

close all;

figure('Name', 'Optimised graph')
show(poseGraph, 'IDs', 'nodes');

% display results
varNames = {'Camera ID', 'tx', 'ty', 'tz', 'qw', 'qx', 'qy', 'qz'};

disp(' ');

resError = sum(edgeResidualErrors(poseGraph), 'all');

disp('Initial guess of camera poses');
fprintf("Sum of residual errors: %d\n", resError);

camNodeData = [(camIDoffset+1:camIDoffset+numCam)', nodes(poseGraph, 1:numCam)];
disp(array2table(camNodeData,'VariableNames',  varNames))

disp(' ');

disp('Optimised camera poses');
resError = sum(edgeResidualErrors(optmPoseGraph), 'all');
fprintf("Sum of residual errors: %d\n", resError);

camNodeData = [nodesMat(1:numCam,1), nodes(optmPoseGraph, 1:numCam)];
disp(array2table(camNodeData,'VariableNames',  varNames))

boardNodeData = [nodesMat(numCam + 1:end,1),nodes(optmPoseGraph, numCam+1:numNodes)];

figure('Name', 'Optimised camera poses')
R = eye(3);
t = [0,0,0];
pose = rigid3d(R,t);
plotCamera('AbsolutePose', pose, 'Size', 0.05, 'AxesVisible', true, 'Label', num2str(camNodeData(1,1))); hold on;
xyzlabel();

%plot cameras
for i = 2:numCam
    R = quat2rotm(camNodeData(i,5:end))';
    t = camNodeData(i,2:4);
    pose = rigid3d(R,t);
    plotCamera('AbsolutePose', pose, 'Size', 0.05, 'AxesVisible', true, 'Label', num2str(camNodeData(i,1)), 'Color', [0,0,1]); hold on;
end

%plot boards
for i=1:size(boardNodeData,1)
    R = quat2rotm(boardNodeData(i,5:end));
    t = boardNodeData(i,2:4);
    pose = rigid3d(R,t);
    trplot(pose.T', 'length', 0.1, 'rviz')
end

grid on;
axis equal;


function poseOut = Convert2MatlabPoseOrder(poseIn)
% Converts order of pose to match expected order in MATLAB.
% INPUT: pose [tx, ty, tz, qx, qy, qz, qw]
% OUTPUT: pose [tx, ty, tz, qw, qx, qy, qz]

poseOut = [poseIn(1:3), poseIn(7), poseIn(4:6)];
end