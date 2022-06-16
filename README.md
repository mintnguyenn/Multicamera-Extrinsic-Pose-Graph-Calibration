# Multi-cameras extrinsic pose graph calibration

**Need to add a description of your ROS package and how to run your it. **

## Nodes and Edges CSV files

- We first want to run the g2o optimisation in MATLAB by creating a .g2o file. We need to create 2 CSV files that contains information of the nodes and edges. 
- The nodes are the pose of each camera w.r.t to the origin camera (camera 7) and the pose of each board at each time-step w.r.t to the origin camera. 
- The edges are the pose estimation of each board at each time-step w.r.t the relevant camera using the aruco board. The `origin node` must be a camera and the `target node` should be a board pose at a single time-step.
- You will need to convert the rotation to Quaternion angles
- More information can be found in the file `nodes_edges_format.ods` (LibreOffice file)



## Main_Optimise_PoseGraph_CSV

This MATLAB script reads the nodes and edges CSV files and performs pose graph optimisation using MATLAB's pose-graph [implementation](https://au.mathworks.com/help/nav/ref/posegraph3d.html). 



## Completed

- ArUco board detection using openCV
- Change git repo name to "multi-camera_extrinsic_pose_graph_calibration"
- Move the Read_ArUco_Yaml to separate file

## To-Do

- [ ] Add the block diagram to readme
- [ ] Synchronisation of camera 7 due to hardware sync. Detect camera 7 images to find the consecutive images of the other cameras. (When camera 7 triggers, the other cameras will trigger)
- [ ] Create function for populating g20 file from given edges. Run g20 separately first.
- [ ] Add g2o graph optimisation to framework
- [ ] Setup the calibration as a pose graph optimisation problem
