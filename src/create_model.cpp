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
#include <pcl_tracking/ObjectCloud.h>

typedef pcl::PointXYZRGBA PointType;
std::string output_filename;

class object_model_creater {
public:
    object_model_creater(ros::NodeHandle nh) : _nh(nh){
        init();
    }
    void init(){
        _sub = _nh.subscribe ("/kinect2/hd/points", 1, &object_model_creater::cloud_cb, this);
        _service = _nh.advertiseService("/get_object_model", &object_model_creater::get_object_model, this);
        ROS_INFO("Ready to get Object Model");
    }

    void removeZeroPoints (const pcl::PointCloud<PointType>::Ptr &cloud,
                           pcl::PointCloud<PointType>::Ptr &result)
    {
      for (size_t i = 0; i < cloud->points.size (); i++)
      {
        PointType point = cloud->points[i];
        if (!(fabs(point.x) < 0.01 &&
              fabs(point.y) < 0.01 &&
              fabs(point.z) < 0.01) &&
            !pcl_isnan(point.x) &&
            !pcl_isnan(point.y) &&
            !pcl_isnan(point.z))
          result->points.push_back(point);
      }

      result->width = static_cast<pcl::uint32_t> (result->points.size ());
      result->height = 1;
      result->is_dense = true;
    }

    void porcess_cloud(){
      _service_response.clear();
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
            tf2::doTransform(_saved_cloud, input_tf, transformStamped);
        }
        catch(tf2::TransformException& ex){
            ROS_ERROR("Received an exception trying to transform a point from \"%s\" to \"%s\": %s", in_frame.c_str(), out_frame.c_str(), ex.what());
        }

        // From PointCloud2 to PCL point cloud
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(input_tf,pcl_pc2);
        pcl::PointCloud<PointType>::Ptr input_tf_pcl(new pcl::PointCloud<PointType>);
        pcl::fromPCLPointCloud2(pcl_pc2,*input_tf_pcl);
        pcl::PointCloud<PointType>::Ptr input_tf_pcl_non_zero (new pcl::PointCloud<PointType>);
        removeZeroPoints(input_tf_pcl, input_tf_pcl_non_zero);

        std::cerr << "PointCloud before filtering: " << input_tf_pcl_non_zero->width * input_tf_pcl_non_zero->height << " data points." << std::endl;

        // Planar filter
        double x_min; _nh.getParam("segm_limits/x/min", x_min);
        double x_max; _nh.getParam("segm_limits/x/max", x_max);
        double y_min; _nh.getParam("segm_limits/y/min", y_min);
        double y_max; _nh.getParam("segm_limits/y/max", y_max);
        double z_min; _nh.getParam("segm_limits/z/min", z_min);
        double z_max; _nh.getParam("segm_limits/z/max", z_max);

        pcl::PointCloud<PointType>::Ptr cloud_filteredZ (new pcl::PointCloud<PointType>);
        pcl::PointCloud<PointType>::Ptr cloud_filteredX (new pcl::PointCloud<PointType>);
        pcl::PointCloud<PointType>::Ptr cloud_filteredY (new pcl::PointCloud<PointType>);
        pcl::PassThrough<PointType> pass;
        pass.setInputCloud (input_tf_pcl_non_zero);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (z_min, z_max);
        pass.filter (*cloud_filteredZ);

        pass.setInputCloud (cloud_filteredZ);
        pass.setFilterFieldName ("y");
        pass.setFilterLimits (y_min, y_max);
        pass.filter (*cloud_filteredY);

        pass.setInputCloud (cloud_filteredY);
        pass.setFilterFieldName ("x");
        pass.setFilterLimits (x_min, x_max);
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

            sensor_msgs::PointCloud2 temp_ros_cloud;
            pcl::toROSMsg(*cloud_cluster, temp_ros_cloud);

            _service_response.push_back(temp_ros_cloud);
            j++;
        }
    }
    bool get_object_model(pcl_tracking::ObjectCloud::Request& req,
                          pcl_tracking::ObjectCloud::Response& res){
      porcess_cloud();
        if(_service_response.empty()){
            ROS_WARN("Objects clouds vector is EMPTY, wait or put object in camera FOV");
            return false;
        }
        else
            res.cloud_vector = _service_response;
        ROS_INFO_STREAM("There are objects clusters, and their number is: " << _service_response.size());
        return true;
    }

    void cloud_cb(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input){
      _saved_cloud = *input;

        
    }


    void spin(){
        ros::spinOnce();
    }


private:
    ros::NodeHandle _nh;
    ros::Subscriber _sub;
    ros::ServiceServer _service;

    sensor_msgs::PointCloud2 _saved_cloud;
    std::vector<sensor_msgs::PointCloud2> _service_response;
};


int
main (int argc, char** argv)
{

    // Initialize ROS
    ros::init (argc, argv, "create_model");
    ros::NodeHandle nh;

    object_model_creater models_creator(nh);

    ros::Rate rate(10);
    // Spin
    while (ros::ok()) {
       models_creator.spin();
       rate.sleep();
    }
    ros::spin ();

}
