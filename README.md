# Physical camera

## KINECT2

roslaunch kinect2_bridge kinect2_bridge.launch

roslaunch pcl_tracking run_create_model.launch

roslaunch pcl_tracking run_tracking.launch

## OPTITRACK

roslaunch mocap_optitrack mocap.launch

roslaunch pcl_tracking optitrack_handler.launch

rostopic echo /visual/obj_pos_vector

# Simulated camera

## KINECT

cd ~/baxter_ws/src/baxter_simulator/baxter_gazebo/launch/

ln -s ~/baxter_ws/src/pcl_tracking/src/simulation/baxter_kinect.urdf.xacro .

ln -s ~/baxter_ws/src/pcl_tracking/src/simulation/baxter_world_with_table.launch .

roslaunch baxter_gazebo baxter_world_with_table.launch

roslaunch pcl_tracking run_create_model.launch real_camera:=false

roslaunch pcl_tracking run_tracking.launch

# Built models

The unsupervised generated models (point clouds) are stored, from bigger to smaller, into the folder "models" with the extension PCD, and it can be observed using:

pcl_viewer ~/baxter_ws/src/pcl_tracking/models/current_timestamp/X.pcd

