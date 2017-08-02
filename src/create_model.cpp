#include <ros/ros.h>
#include <iostream>
#include <string>
#include <sstream>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/transform_datatypes.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

typedef pcl::PointXYZRGBA PointType;
std::string output_filename;

void cloud_cb(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input){

  // Transform PointCloud2 from kinect frame to base frame
  sensor_msgs::PointCloud2 input_tf;  

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener listener(tfBuffer);
  std::string in_frame = "base";
  std::string out_frame = "kinect2_link";
  geometry_msgs::TransformStamped transformStamped;

  try{
    ros::Time now = ros::Time(0);
    transformStamped = tfBuffer.lookupTransform(in_frame, 
                                                out_frame,
                                                ros::Time(0), 
                                                ros::Duration(10.0));
    std::cout << transformStamped << std::endl;
    tf2::doTransform(*input, input_tf, transformStamped);
  }
  catch(tf2::TransformException& ex){
      ROS_ERROR("Received an exception trying to transform a point from \"%s\" to \"%s\": %s", in_frame.c_str(), out_frame.c_str(), ex.what());
  }

  // From PointCloud2 to PCL point cloud
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(input_tf,pcl_pc2);
  pcl::PointCloud<PointType>::Ptr input_tf_pcl(new pcl::PointCloud<PointType>);
  pcl::fromPCLPointCloud2(pcl_pc2,*input_tf_pcl); 

  std::cerr << "PointCloud before filtering: " << input_tf_pcl->width * input_tf_pcl->height << " data points." << std::endl;

  // Create the filtering object
  pcl::PointCloud<PointType>::Ptr cloud_filteredZ (new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>::Ptr cloud_filteredX (new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>::Ptr cloud_filteredY (new pcl::PointCloud<PointType>);
  pcl::PassThrough<PointType> pass;
  pass.setInputCloud (input_tf_pcl);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (-0.19, 0.2);
  pass.filter (*cloud_filteredZ);

  pass.setInputCloud (cloud_filteredZ);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (-0.25, 0.25);
  pass.filter (*cloud_filteredY);

  pass.setInputCloud (cloud_filteredY);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (0.65, 0.95);
  pass.filter (*cloud_filteredX);

  std::cerr << "PointCloud after filtering: " << cloud_filteredX->width * cloud_filteredX->height << " data points." << std::endl;

  ///// Transform back the reference frame

  // From PCL point cloud to PointCloud2
  sensor_msgs::PointCloud2 tmp_cloud_ros, tmp_cloud_ros_tf;
  pcl::toROSMsg(*cloud_filteredX, tmp_cloud_ros);

  // change frame
  try{
    ros::Time now = ros::Time(0);
    transformStamped = tfBuffer.lookupTransform(out_frame, 
                                                in_frame,
                                                ros::Time(0), 
                                                ros::Duration(10.0));
    //std::cout << transformStamped << std::endl;
    tf2::doTransform(tmp_cloud_ros, tmp_cloud_ros_tf, transformStamped);
  }
  catch(tf2::TransformException& ex){
      ROS_ERROR("Received an exception trying to transform a point from \"%s\" to \"%s\": %s", in_frame.c_str(), out_frame.c_str(), ex.what());
  }

  // From PointCloud2 to PCL point cloud
  pcl::PointCloud<PointType>::Ptr tmp_cloud_pcl_tf(new pcl::PointCloud<PointType>);
  pcl_conversions::toPCL(tmp_cloud_ros_tf,pcl_pc2);
  pcl::fromPCLPointCloud2(pcl_pc2,*tmp_cloud_pcl_tf); 

  // // Write the original version to disk
  pcl::PCDWriter writer;
  writer.write<PointType> ("/home/maestre/baxter_ws/src/pcl_tracking/src/original.pcd", *input_tf_pcl, false);

  // Write the downsampled version to disk  
  writer.write<PointType> (output_filename, *tmp_cloud_pcl_tf, false);

  ros::shutdown();
}

int
main (int argc, char** argv)
{
  if (argc < 2)
    {
      PCL_WARN("Please set model name: roslaunch pcl_tracking run_create_model.launch model_name:=name \n");
      exit (1);
    }
  else 
    {
      // std::ostringstream oss;
      // oss << "src/pcl_tracking/src/" + std::string (argv[1]) + ".pcd";
      // output_filename = oss.str();
      output_filename = std::string (argv[1]);
    }

  // Initialize ROS
  ros::init (argc, argv, "create_model");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/kinect2/hd/points", 1, cloud_cb);
  // ros::Subscriber sub = nh.subscribe<> ("/kinect2/hd/points", 1, 
  //                                     boost::bind(cloud_cb, _1, output_filename));  

  // Spin
  ros::spin ();

}