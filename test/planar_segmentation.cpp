#include <ros/ros.h>
// PCL specific includes
#include <pcl/io/pcd_io.h>
#include <pcl/console/time.h>
//header for filtering
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
//header for clustering purpose
#include <pcl/segmentation/conditional_euclidean_clustering.h>
//header for subscribing to kinect
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//header for model coeffs-->optional
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
//header planar segmentation 
#include <pcl/segmentation/sac_segmentation.h>

ros::Publisher pub;
ros::Publisher coef_pub, ind_pub;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input){

        // ROS_ERROR("Inside CB");

        pcl::PointCloud<pcl::PointXYZ> cloud;
      	pcl::PointCloud<pcl::PointXYZ> cloud_filtered;
      	pcl::PointCloud<pcl::PointXYZ> voxel_filtered;
        pcl::PointCloud<pcl::PointXYZ> cloud_segmented;
        pcl::PointCloud<pcl::PointXYZ> planar_object;
        pcl::PointCloud<pcl::PointXYZ> in_future;

        pcl::fromROSMsg(*input, cloud);

        pcl::ModelCoefficients coefficients;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

        // Create the segmentation object
        pcl::SACSegmentation<pcl::PointXYZ> segmentation;
        segmentation.setModelType(pcl::SACMODEL_PLANE);
        segmentation.setMethodType(pcl::SAC_RANSAC);
        segmentation.setMaxIterations(1000);
        segmentation.setDistanceThreshold(0.015);
        segmentation.setInputCloud(cloud.makeShared());
        segmentation.segment(*inliers, coefficients);

//        // Publish the model coefficients
//        pcl_msgs::ModelCoefficients ros_coefficients;
//        pcl_conversions::fromPCL(coefficients, ros_coefficients);
//        coef_pub.publish(ros_coefficients);

//        // Publish the Point Indices
//        pcl_msgs::PointIndices ros_inliers;
//        pcl_conversions::fromPCL(*inliers, ros_inliers);
//        ind_pub.publish(ros_inliers);

        // Create the filtering object
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud.makeShared());
        extract.setIndices(inliers);
        extract.setNegative(false);
//        extract.filter(cloud_segmented);
        extract.filter(planar_object);


        // Extract the planar object
        extract.setNegative(true);
        extract.filter(in_future);

        //Publish the new cloud
        sensor_msgs::PointCloud2 output;
//        pcl::toROSMsg(cloud_segmented, output);
        pcl::toROSMsg(in_future, output);
        pub.publish(output);

        ROS_ERROR("Publishing");
    }


int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "new_tuto");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/kinect2/qhd/points", 1, cloud_cb);

  // Create a ROS publisher for the output model coefficients
  //pub = nh.advertise<pcl_msgs::ModelCoefficients> ("output", 1);
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
  coef_pub = nh.advertise<pcl_msgs::ModelCoefficients>("planar_coef", 1);
  ind_pub = nh.advertise<pcl_msgs::PointIndices>("point_indices", 1);
  // Spin
  ros::spin ();
}
