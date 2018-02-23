#include <pcl/point_types.h>
#include <ros/ros.h>
#include <pcl/features/normal_3d.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud){
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*cloud, pcl_pc2);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2 (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(pcl_pc2, *cloud_2);


        // Create the normal estimation class, and pass the input dataset to it
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        ne.setInputCloud (cloud_2);

        // Create an empty kdtree representation, and pass it to the normal estimation object.
        // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
        ne.setSearchMethod (tree);

        // Output datasets
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

        // Use all neighbors in a sphere of radius 3cm
        ne.setRadiusSearch (0.03);

        // Compute the features
        ne.compute (*cloud_normals);
        ROS_WARN_STREAM("NORMAL CLOUD SIZE IS : " << cloud_normals->points.size());

        // cloud_normals->points.size () should have the same size as the input cloud->points.size ()
        // Write the original version to disk
        pcl::PCDWriter writer;
        std::string path_filtered_pcd = "/home/ghanim/git/pcl_tracking/test/";
        writer.write<pcl::Normal> (path_filtered_pcd + "filtered.pcd", *cloud_normals, false);
    }

int main(int argc, char** argv)
    {
        ros::init(argc, argv, "normal_estimation_node");
        ros::NodeHandle nh;
        ros::Subscriber point_cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth_registered/sw_registered/points", 1, cloud_cb);

        ros::AsyncSpinner my_spinner(1);
        my_spinner.start();

        ros::waitForShutdown();

    }
