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

#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

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
    // std::cout << transformStamped << std::endl;
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

  // Planar filter
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

  std::cerr << "PointCloud after planar filtering: " << cloud_filteredX->width * cloud_filteredX->height << " data points." << std::endl;

  // Euclidean filter
  pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA>);
  tree->setInputCloud (cloud_filteredX);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (500);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filteredX);
  ec.extract (cluster_indices);

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

  std::cerr << "Number of clusters: " << cluster_indices.size() << std::endl;
  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGBA>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->points.push_back (tmp_cloud_pcl_tf->points[*pit]);
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    std::stringstream ss;
    ss << "/home/maestre/Desktop/cluster_examples/cloud_cluster_" << j << ".pcd";
    writer.write<pcl::PointXYZRGBA> (ss.str (), *cloud_cluster, false); //*
    j++;
  }  

  ros::shutdown();
}

int
main (int argc, char** argv)
{

  // Initialize ROS
  ros::init (argc, argv, "create_model");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/kinect2/hd/points", 1, cloud_cb);

  // Spin
  ros::spin ();

}