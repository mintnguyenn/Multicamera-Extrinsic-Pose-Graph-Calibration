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
%and accounts for sensor noise)
inforMatDefault = [1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 1 0 0 1 0 1];

%the reference node
fromNode = 1;

%add nodes to posegraph
for i = 2:numNodes
    curPose = Convert2MatlabPoseOrder(nodesMat(i,2:end));
    addRelativePose(poseGraph, curPose, inforMatDefault, fromNode);
end

%add edges to posegraph
for i = 1:numEdges
    curPose = Convert2MatlabPoseOrder(edgesMat(i,3:end));
    camNode = edgesMat(i,1) - camIDoffset;
    boardNode = edgesMat(i,2) - boardIDoffset;
    addRelativePose(poseGraph, curPose, inforMatDefault, camNode, boardNode);
end

%plot graph before optimising
fname = 'graph_before_after_optimising';
hfig = figure('Name', 'Before Optimisation: nodes and edges');
tiledlayout(2,1, 'Padding', 'compact');

ax1 = nexttile;
show(poseGraph, 'IDs', 'off'); hold on;
xlabel('x (m)')
ylabel('y (m)')
zlabel('z (m)')
title('Before Optimisation');

camNodeData = [(camIDoffset+1:camIDoffset+numCam)', nodes(poseGraph, 1:numCam)];

%plot cameras
for i = 1:numCam
    t = camNodeData(i,2:4);
    cam = camNodeData(i,1);
    text(t(1)+0.02, t(2)-0.02, t(3), num2str(cam), 'Color', 'blue', 'Clipping', 'on');

end

axis auto;

fprintf("DONE\n")

%% Perform optimisation

fprintf("Optimising pose graph ...")
optmPoseGraph = optimizePoseGraph(poseGraph);
fprintf("DONE\n")

%% Show results

% plot graph after optimising
ax2 = nexttile;
show(optmPoseGraph, 'IDs', 'off');
xlabel('x (m)')
ylabel('y (m)')
zlabel('z (m)')
title('After Optimisation');

camNodeData = [(camIDoffset+1:camIDoffset+numCam)', nodes(optmPoseGraph, 1:numCam)];

for i = 1:numCam
    t = camNodeData(i,2:4);
    cam = camNodeData(i,1);
    text(t(1)+0.02, t(2)-0.02, t(3), num2str(cam), 'Color', 'blue', 'Clipping', 'on');
end

linkaxes([ax1 ax2])
axis auto;

picturewidth = 20; % set this parameter and keep it forever
hw_ratio = 1.2; % feel free to play with this ratio

% set(findall(hfig,'-property','Box'),'Box','on') % optional
set(findall(hfig,'-property','FontSize'),'FontSize',14) % adjust fontsize to your document
set(findall(hfig,'-property','Interpreter'),'Interpreter','latex') 
set(findall(hfig,'-property','TickLabelInterpreter'),'TickLabelInterpreter','latex')
set(hfig,'Units','centimeters','Position',[3 3 picturewidth hw_ratio*picturewidth])
pos = get(hfig,'Position');
set(hfig,'PaperPositionMode','Auto','PaperUnits','centimeters','PaperSize',[pos(3), pos(4)])
print(hfig,fullfile(csvDir, fname),'-dpng','-painters')

close all;


% display results in table
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

fname = 'optimised_pose_graph.png';
hfig = figure('Name', 'Optimised camera poses');
R = eye(3);
t = [0,0,0];
pose = rigid3d(R,t);
plotCamera('AbsolutePose', pose, 'Size', 0.05, 'AxesVisible', false, 'Label', num2str(camNodeData(1,1))); hold on;
trplot(pose.T', 'length', 0.1, 'thick', 2, 'rgb', 'notext', 'noarrow')
xlabel('x (m)')
ylabel('y (m)')
zlabel('z (m)')
set(findall(hfig,'-property','FontSize'),'FontSize',14) % adjust fontsize to your document


%plot cameras
for i = 2:numCam
    R = quat2rotm(camNodeData(i,5:end))';
    t = camNodeData(i,2:4);
    pose = rigid3d(R,t);
    plotCamera('AbsolutePose', pose, 'Size', 0.05, 'AxesVisible', false, 'Label', num2str(camNodeData(i,1)), 'Color', [0,0,1]); hold on;
    trplot(pose.T', 'length', 0.1, 'thick', 2, 'rgb', 'notext', 'noarrow')
end

%plot boards
for i=1:10:size(boardNodeData,1)
    R = quat2rotm(boardNodeData(i,5:end));
    t = boardNodeData(i,2:4);
    pose = rigid3d(R,t);
    trplot(pose.T', 'length', 0.05, 'thick', 2, 'rgb', 'notext', 'noarrow')
    text(t(1), t(2), t(3), num2str(i));
end

grid on;
axis equal;
view(5,-45)

picturewidth = 20; % set this parameter and keep it forever
hw_ratio = 0.6; % feel free to play with this ratio

% set(findall(hfig,'-property','Box'),'Box','on') % optional
set(findall(hfig,'-property','Interpreter'),'Interpreter','latex') 
set(findall(hfig,'-property','TickLabelInterpreter'),'TickLabelInterpreter','latex')
set(hfig,'Units','centimeters','Position',[3 3 picturewidth hw_ratio*picturewidth])
pos = get(hfig,'Position');
set(hfig,'PaperPositionMode','Auto','PaperUnits','centimeters','PaperSize',[pos(3), pos(4)])
print(hfig,fullfile(csvDir, fname),'-dpng','-painters')





function poseOut = Convert2MatlabPoseOrder(poseIn)
% Converts order of pose to match expected order in MATLAB.
% INPUT: pose [tx, ty, tz, qx, qy, qz, qw]
% OUTPUT: pose [tx, ty, tz, qw, qx, qy, qz]

poseOut = [poseIn(1:3), poseIn(7), poseIn(4:6)];
end