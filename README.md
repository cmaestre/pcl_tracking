# pcl tracking

KINECT2

roslaunch kinect2_bridge kinect2_bridge.launch

roslaunch pcl_tracking run_create_model.launch

roslaunch pcl_tracking run_tracking.launch

-------------------------- 
OPTITRACK

roslaunch mocap_optitrack mocap.launch

roslaunch pcl_tracking optitrack_handler.launch

rostopic echo /visual/obj_pos_vector