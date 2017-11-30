#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>

void
cloud_cb (const sensor_msgs::PointCloud2::ConstPtr& cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*cloud, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2, *pcl_cloud);
    // Create the filtering object: downsample the dataset using a leaf size of 1cm
      pcl::VoxelGrid<pcl::PointXYZ> vg;
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
      vg.setInputCloud (pcl_cloud);
      vg.setLeafSize (0.01f, 0.01f, 0.01f);
      vg.filter (*cloud_filtered);
      std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*

      // Create the segmentation object for the planar model and set all the parameters
      pcl::SACSegmentation<pcl::PointXYZ> seg;
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
      pcl::PCDWriter writer;
      seg.setOptimizeCoefficients (true);
      seg.setModelType (pcl::SACMODEL_PLANE);
      seg.setMethodType (pcl::SAC_RANSAC);
      seg.setMaxIterations (100);
      seg.setDistanceThreshold (0.02);

      int i=0, nr_points = (int) cloud_filtered->points.size ();
      while (cloud_filtered->points.size () > 0.3 * nr_points)
      {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (cloud_filtered);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
          std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
          break;
        }

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud (cloud_filtered);
        extract.setIndices (inliers);
        extract.setNegative (false);

        // Get the points associated with the planar surface
        extract.filter (*cloud_plane);
        std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

        // Remove the planar inliers, extract the rest
        extract.setNegative (true);
        extract.filter (*cloud_f);
        *cloud_filtered = *cloud_f;
      }

      // Creating the KdTree object for the search method of the extraction
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
      tree->setInputCloud (cloud_filtered);

      std::vector<pcl::PointIndices> cluster_indices;
      pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
      ec.setClusterTolerance (0.02); // 2cm
      ec.setMinClusterSize (100);
      ec.setMaxClusterSize (25000);
      ec.setSearchMethod (tree);
      ec.setInputCloud (cloud_filtered);
      ec.extract (cluster_indices);

      int j = 0;
      for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
      {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
          cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
        std::stringstream ss;
        ss << "cloud_cluster_" << j << ".pcd";
        writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
        j++;
      }



//    std::cerr << "Point cloud data: " << pcl_cloud->points.size () << " points" << std::endl;
//    //    for (size_t i = 0; i < pcl_cloud->points.size (); ++i)
//    //        std::cerr << "    " << pcl_cloud->points[i].x << " "
//    //                  << pcl_cloud->points[i].y << " "
//    //                  << pcl_cloud->points[i].z << std::endl;

//    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
//    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
//    // Create the segmentation object
//    pcl::SACSegmentation<pcl::PointXYZ> seg;
//    // Optional
//    seg.setOptimizeCoefficients (true);
//    // Mandatory
//    seg.setModelType (pcl::SACMODEL_PLANE);
//    seg.setMethodType (pcl::SAC_RANSAC);
//    seg.setDistanceThreshold (0.01);

//    seg.setInputCloud (pcl_cloud);
//    seg.segment (*inliers, *coefficients);

//    if (inliers->indices.size () == 0)
//    {
//        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
//        return ;
//    }

//    std::cerr << "Model coefficients: " << coefficients->values[0] << " "
//              << coefficients->values[1] << " "
//              << coefficients->values[2] << " "
//              << coefficients->values[3] << std::endl;

//    std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;

//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::PCDWriter writer;
//    // Extract the planar inliers from the input cloud
//    pcl::ExtractIndices<pcl::PointXYZ> extract;
//    extract.setInputCloud (cloud_filtered);
//    extract.setIndices (inliers);
//    extract.setNegative (false);

//    // Get the points associated with the planar surface
//    extract.filter (*cloud_plane);
//    std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

//    // Remove the planar inliers, extract the rest
//    extract.setNegative (true);
//    extract.filter (*cloud_f);
//    *cloud_filtered = *cloud_f;
    //    for (size_t i = 0; i < inliers->indices.size (); ++i)
    //        std::cerr << inliers->indices[i] << "    " << pcl_cloud->points[inliers->indices[i]].x << " "
    //                  << pcl_cloud->points[inliers->indices[i]].y << " "
    //                  << pcl_cloud->points[inliers->indices[i]].z << std::endl;


}

int
main (int argc, char** argv)
{   

    // Initialize ROS
    ros::init (argc, argv, "planar_segmentation_node");
    ros::NodeHandle nh;
    XmlRpc::XmlRpcValue parameters;
    nh.getParam("/", parameters);
    std::string camera_type = static_cast<std::string>(parameters["camera_type"]);

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub;
    if(camera_type == "kinect_2")
        sub = nh.subscribe ("/kinect2/hd/points", 1, cloud_cb);
    if(camera_type == "kinect_1")
        sub = nh.subscribe ("/camera/depth_registered/sw_registered/points", 1, cloud_cb);

    ros::spin();
    ros::waitForShutdown();

    return (0);
}
