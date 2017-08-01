/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <ros/ros.h>
#include <iostream>
#include <string>
#include <sensor_msgs/PointCloud2.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/transform_datatypes.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h> 

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h> 

#include <pcl/tracking/tracking.h>
#include <pcl/tracking/particle_filter.h>
#include <pcl/tracking/kld_adaptive_particle_filter_omp.h>
#include <pcl/tracking/particle_filter_omp.h>

#include <pcl/tracking/coherence.h>
#include <pcl/tracking/distance_coherence.h>
#include <pcl/tracking/hsv_color_coherence.h>
#include <pcl/tracking/normal_coherence.h>

#include <pcl/tracking/approx_nearest_pair_point_cloud_coherence.h>
#include <pcl/tracking/nearest_pair_point_cloud_coherence.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni2_grabber.h>
#include <pcl/console/parse.h>
#include <pcl/common/time.h>
#include <pcl/common/centroid.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/io/pcd_io.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/integral_image_normal.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/surface/convex_hull.h>

#include <pcl/search/pcl_search.h>
#include <pcl/common/transforms.h>

#include <boost/format.hpp>

int nb_wait_iter = 2;

#define FPS_CALC_BEGIN                          \
    static double duration = 0;                 \
    double start_time = pcl::getTime ();        \

#define FPS_CALC_END(_WHAT_)                    \
  {                                             \
    double end_time = pcl::getTime ();          \
    static unsigned count = 0;                  \
    if (++count == nb_wait_iter)                          \
    {                                           \
      std::cout << "Average framerate("<< _WHAT_ << "): " << double(count)/double(duration) << " Hz" <<  std::endl; \
      count = 0;                                                        \
      duration = 0.0;                                                   \
    }                                           \
    else                                        \
    {                                           \
      duration += end_time - start_time;        \
    }                                           \
  }

using namespace pcl::tracking;

template <typename PointType>
class OpenNISegmentTracking
{
public:
  //typedef pcl::PointXYZRGBANormal RefPointType;
  typedef pcl::PointXYZRGBA RefPointType;
  //typedef pcl::PointXYZ RefPointType;
  typedef ParticleXYZRPY ParticleT;
  
  typedef pcl::PointCloud<PointType> Cloud;
  typedef pcl::PointCloud<RefPointType> RefCloud;
  typedef typename RefCloud::Ptr RefCloudPtr;
  typedef typename RefCloud::ConstPtr RefCloudConstPtr;
  typedef typename Cloud::Ptr CloudPtr;
  typedef typename Cloud::ConstPtr CloudConstPtr;
  //typedef KLDAdaptiveParticleFilterTracker<RefPointType, ParticleT> ParticleFilter;
  //typedef KLDAdaptiveParticleFilterOMPTracker<RefPointType, ParticleT> ParticleFilter;
  //typedef ParticleFilterOMPTracker<RefPointType, ParticleT> ParticleFilter;
  typedef ParticleFilterTracker<RefPointType, ParticleT> ParticleFilter;
  typedef typename ParticleFilter::CoherencePtr CoherencePtr;
  typedef typename pcl::search::KdTree<PointType> KdTree;
  typedef typename KdTree::Ptr KdTreePtr;
  OpenNISegmentTracking (const std::string& device_id, int thread_nr, double downsampling_grid_size,
                         bool use_convex_hull,
                         bool visualize_non_downsample, bool visualize_particles,
                         bool use_fixed)
  : viewer_ ("PCL OpenNI Tracking Viewer")
  , device_id_ (device_id)
  , new_cloud_ (false)
  , ne_ (thread_nr)
  , counter_ (0)
  , use_convex_hull_ (use_convex_hull)
  , visualize_non_downsample_ (visualize_non_downsample)
  , visualize_particles_ (visualize_particles)
  , downsampling_grid_size_ (downsampling_grid_size)
  {
    KdTreePtr tree (new KdTree (false));
    ne_.setSearchMethod (tree);
    ne_.setRadiusSearch (0.03);
    
    std::vector<double> default_step_covariance = std::vector<double> (6, 0.015 * 0.015);
    default_step_covariance[3] *= 40.0;
    default_step_covariance[4] *= 40.0;
    default_step_covariance[5] *= 40.0;
    
    std::vector<double> initial_noise_covariance = std::vector<double> (6, 0.00001);
    std::vector<double> default_initial_mean = std::vector<double> (6, 0.0);
    if (use_fixed)
    {
      boost::shared_ptr<ParticleFilterOMPTracker<RefPointType, ParticleT> > tracker
        (new ParticleFilterOMPTracker<RefPointType, ParticleT> (thread_nr));
      tracker_ = tracker;
    }
    else
    {
      boost::shared_ptr<KLDAdaptiveParticleFilterOMPTracker<RefPointType, ParticleT> > tracker
        (new KLDAdaptiveParticleFilterOMPTracker<RefPointType, ParticleT> (thread_nr));
      tracker->setMaximumParticleNum (500);
      tracker->setDelta (0.99);
      tracker->setEpsilon (0.2);
      ParticleT bin_size;
      bin_size.x = 0.1f;
      bin_size.y = 0.1f;
      bin_size.z = 0.1f;
      bin_size.roll = 0.1f;
      bin_size.pitch = 0.1f;
      bin_size.yaw = 0.1f;
      tracker->setBinSize (bin_size);
      tracker_ = tracker;
    }
    
    tracker_->setTrans (Eigen::Affine3f::Identity ());
    tracker_->setStepNoiseCovariance (default_step_covariance);
    tracker_->setInitialNoiseCovariance (initial_noise_covariance);
    tracker_->setInitialNoiseMean (default_initial_mean);
    tracker_->setIterationNum (1);
    
    tracker_->setParticleNum (400);
    tracker_->setResampleLikelihoodThr(0.00);
    tracker_->setUseNormal (false);
    // setup coherences
    ApproxNearestPairPointCloudCoherence<RefPointType>::Ptr coherence = ApproxNearestPairPointCloudCoherence<RefPointType>::Ptr
      (new ApproxNearestPairPointCloudCoherence<RefPointType> ());
    // NearestPairPointCloudCoherence<RefPointType>::Ptr coherence = NearestPairPointCloudCoherence<RefPointType>::Ptr
    //   (new NearestPairPointCloudCoherence<RefPointType> ());
    
    boost::shared_ptr<DistanceCoherence<RefPointType> > distance_coherence
      = boost::shared_ptr<DistanceCoherence<RefPointType> > (new DistanceCoherence<RefPointType> ());
    coherence->addPointCoherence (distance_coherence);
    
    boost::shared_ptr<HSVColorCoherence<RefPointType> > color_coherence
      = boost::shared_ptr<HSVColorCoherence<RefPointType> > (new HSVColorCoherence<RefPointType> ());
    color_coherence->setWeight (0.1);
    coherence->addPointCoherence (color_coherence);
    
    //boost::shared_ptr<pcl::search::KdTree<RefPointType> > search (new pcl::search::KdTree<RefPointType> (false));
    boost::shared_ptr<pcl::search::Octree<RefPointType> > search (new pcl::search::Octree<RefPointType> (0.01));
    //boost::shared_ptr<pcl::search::OrganizedNeighbor<RefPointType> > search (new pcl::search::OrganizedNeighbor<RefPointType>);
    coherence->setSearchMethod (search);
    coherence->setMaximumDistance (0.01);
    tracker_->setCloudCoherence (coherence);
  }

  bool
  drawParticles (pcl::visualization::PCLVisualizer& viz)
  {
    ParticleFilter::PointCloudStatePtr particles = tracker_->getParticles ();
    if (particles)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr particle_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
      for (size_t i = 0; i < particles->points.size (); i++)
      {
        pcl::PointXYZ point;
        
        point.x = particles->points[i].x;
        point.y = particles->points[i].y;
        point.z = particles->points[i].z;
        particle_cloud->points.push_back (point);
      }
      if (visualize_particles_)
      {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> blue_color (particle_cloud, 250, 99, 71);
        if (!viz.updatePointCloud (particle_cloud, blue_color, "particle cloud"))
          viz.addPointCloud (particle_cloud, blue_color, "particle cloud");
      }      
      return true;
    }
    else
    {
      PCL_WARN ("no particles\n");
      return false;
    }
  }
  
  void
  drawResult (pcl::visualization::PCLVisualizer& viz)
  {
    ParticleXYZRPY result = tracker_->getResult ();
    Eigen::Affine3f transformation = tracker_->toEigenMatrix (result);
    // move a little bit for better visualization
    transformation.translation () += Eigen::Vector3f (0.0f, 0.0f, -0.005f);
    RefCloudPtr result_cloud (new RefCloud ());

    //pcl::transformPointCloud<RefPointType> (*(tracker_->getReferenceCloud ()), *result_cloud, transformation);
    pcl::transformPointCloud<RefPointType> (*reference_, *result_cloud, transformation);

    if (visualize_particles_)
    {
      pcl::visualization::PointCloudColorHandlerCustom<RefPointType> red_color (result_cloud, 0, 0, 255);
      if (!viz.updatePointCloud (result_cloud, red_color, "resultcloud"))
        viz.addPointCloud (result_cloud, red_color, "resultcloud");
    }
    tracked_cloud_ = result_cloud;
  }
  
  void
  viz_cb (pcl::visualization::PCLVisualizer& viz)
  {
    // ROS_ERROR("Inside viz_cb");

    // boost::mutex::scoped_lock lock (mtx_);

    // set initial position of the camera
    viz.setBackgroundColor(0, 0, 0);
    viz.setCameraClipDistances(0.0106913, 2);
    // viz.setCameraPosition( 0.22543, 0.942367, -1.04499, 0.0702978, 0.317386, -0.0205721, 0.0647538, -0.85621, -0.512554);
    viz.setCameraPosition( 0, 0, 1, 0.25, 0, 0.57, 0.43, 0, 1.25);
    viz.setCameraFieldOfView(1);
    viz.setSize(960, 716);
    viz.setPosition(250, 52);
      
    if (!cloud_pass_)
    {
      boost::this_thread::sleep (boost::posix_time::seconds (1));
      return;
    }

    if (new_cloud_ && cloud_pass_downsampled_)
    {
      CloudPtr cloud_pass;
      cloud_pass = cloud_pass_;
      
      if (!viz.updatePointCloud (cloud_pass, "cloudpass"))
        {
          viz.addPointCloud (cloud_pass, "cloudpass");
          viz.resetCameraViewpoint ("cloudpass");
        }
    }

    if (new_cloud_ && reference_)
    {
      bool ret = drawParticles (viz);
      if (ret)
      {
        drawResult (viz);
        
        // draw some texts
        viz.removeShape ("N");
        viz.addText ((boost::format ("number of Reference PointClouds: %d") % tracker_->getReferenceCloud ()->points.size ()).str (),
                     10, 20, 20, 1.0, 1.0, 1.0, "N");
        
        viz.removeShape ("M");
        viz.addText ((boost::format ("number of Measured PointClouds:  %d") % cloud_pass_downsampled_->points.size ()).str (),
                     10, 40, 20, 1.0, 1.0, 1.0, "M");
        
        viz.removeShape ("tracking");
        viz.addText ((boost::format ("tracking:        %f fps") % (1.0 / tracking_time_)).str (),
                     10, 60, 20, 1.0, 1.0, 1.0, "tracking");
        
        viz.removeShape ("downsampling");
        viz.addText ((boost::format ("downsampling:    %f fps") % (1.0 / downsampling_time_)).str (),
                     10, 80, 20, 1.0, 1.0, 1.0, "downsampling");
        
        viz.removeShape ("computation");
        viz.addText ((boost::format ("computation:     %f fps") % (1.0 / computation_time_)).str (),
                     10, 100, 20, 1.0, 1.0, 1.0, "computation");

        viz.removeShape ("particles");
        viz.addText ((boost::format ("particles:     %d") % tracker_->getParticles ()->points.size ()).str (),
                     10, 120, 20, 1.0, 1.0, 1.0, "particles");
        
      }
    }
    new_cloud_ = false;

    ///// Bounding box

    viz.removeShape("cube");

    if (tracked_cloud_)
    {
      // Remove previous elements
      viz.removeShape("cube");
      viz.removeCoordinateSystem();

      // Bounding box 
      // compute principal direction
      Eigen::Vector4f centroid;
      pcl::compute3DCentroid(*tracked_cloud_, centroid);
      Eigen::Matrix3f covariance;
      computeCovarianceMatrixNormalized(*tracked_cloud_, centroid, covariance);
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
      Eigen::Matrix3f eigDx = eigen_solver.eigenvectors();
      eigDx.col(2) = eigDx.col(0).cross(eigDx.col(1));

      // move the points to the reference frame
      Eigen::Matrix4f p2w(Eigen::Matrix4f::Identity());
      p2w.block<3,3>(0,0) = eigDx.transpose();
      p2w.block<3,1>(0,3) = -1.f * (p2w.block<3,3>(0,0) * centroid.head<3>());
      pcl::PointCloud<RefPointType> cPoints;
      pcl::transformPointCloud(*tracked_cloud_, cPoints, p2w);

      RefPointType min_pt, max_pt;
      pcl::getMinMax3D(cPoints, min_pt, max_pt);
      const Eigen::Vector3f mean_diag = 0.5f*(max_pt.getVector3fMap() + min_pt.getVector3fMap());
      // final transform
      const Eigen::Quaternionf qfinal(eigDx);
      const Eigen::Vector3f tfinal = eigDx*mean_diag + centroid.head<3>();

      viz.addCube(tfinal, qfinal, max_pt.x - min_pt.x, max_pt.y - min_pt.y, max_pt.z - min_pt.z);
      viz.setRepresentationToWireframeForAllActors();

      // Compute position and orientation
      // Multiply centroid by transformation matrix
      Eigen::Vector3f centroids = centroid.head<3>();
      ROS_ERROR_STREAM("Tracked object position: " <<  
                        centroids[0] << " " << 
                        centroids[1] << " " << 
                        centroids[2]);
      Eigen::Affine3f trans; // = Eigen::Affine3f::Identity ();
      trans = eigDx;
      trans.translation ().matrix () = Eigen::Vector3f (centroids[0], centroids[1], centroids[2]);
      viz.addCoordinateSystem (0.25, trans);

    }

  }

  void filterPassThrough (const CloudConstPtr &cloud, Cloud &result)
  {
    FPS_CALC_BEGIN;
    pcl::PassThrough<PointType> pass;
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (-1, 1);
    //pass.setFilterLimits (0.0, 1.5);
    //pass.setFilterLimits (0.0, 0.6);
    pass.setKeepOrganized (false);
    pass.setInputCloud (cloud);
    pass.filter (result);
    FPS_CALC_END("filterPassThrough");
  }
  
  void gridSample (const CloudConstPtr &cloud, Cloud &result, double leaf_size = 0.01)
  {
    FPS_CALC_BEGIN;
    double start = pcl::getTime ();
    pcl::VoxelGrid<PointType> grid;
    //pcl::ApproximateVoxelGrid<PointType> grid;
    grid.setLeafSize (float (leaf_size), float (leaf_size), float (leaf_size));
    grid.setInputCloud (cloud);
    grid.filter (result);
    //result = *cloud;
    double end = pcl::getTime ();
    downsampling_time_ = end - start;
    FPS_CALC_END("gridSample");
  }
  
  void gridSampleApprox (const CloudConstPtr &cloud, Cloud &result, double leaf_size = 0.01)
  {
    FPS_CALC_BEGIN;
    double start = pcl::getTime ();
    //pcl::VoxelGrid<PointType> grid;
    pcl::ApproximateVoxelGrid<PointType> grid;
    grid.setLeafSize (static_cast<float> (leaf_size), static_cast<float> (leaf_size), static_cast<float> (leaf_size));
    grid.setInputCloud (cloud);
    grid.filter (result);
    //result = *cloud;
    double end = pcl::getTime ();
    downsampling_time_ = end - start;
    FPS_CALC_END("gridSample");
  }

  void removeZeroPoints (const CloudConstPtr &cloud,
                         Cloud &result)
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
        result.points.push_back(point);
    }

    result.width = static_cast<pcl::uint32_t> (result.points.size ());
    result.height = 1;
    result.is_dense = true;
  }

  void
  cloud_cb (const boost::shared_ptr<const sensor_msgs::PointCloud2>& cloud)
  {
    boost::mutex::scoped_lock lock (mtx_);

    // ROS_ERROR("Inside cloud_cb");

    std::cerr << "cloud : " << cloud->width * cloud->height << " data points." << std::endl;
    
    double start = pcl::getTime ();
    FPS_CALC_BEGIN;

    // Transform PointCloud2 from kinect frame to base frame
    sensor_msgs::PointCloud2 cloud_tf;  

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener listener(tfBuffer);
    std::string in_frame = "base";
    std::string out_frame = "kinect2_rgb_optical_frame";
    geometry_msgs::TransformStamped transformStamped;

    try{
      ros::Time now = ros::Time::now();
      transformStamped = tfBuffer.lookupTransform(in_frame,
                                                  out_frame,
                                                  ros::Time(0),
                                                  ros::Duration(3.0));
      //std::cout << transformStamped << std::endl;
      tf2::doTransform(*cloud, cloud_tf, transformStamped);
    }
    catch(tf2::TransformException& ex){
        ROS_ERROR("Received an exception trying to transform a point from \"%s\" to \"%s\": %s", in_frame.c_str(), out_frame.c_str(), ex.what());
    }

    // From PointCloud2 to PCL point cloud
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(cloud_tf,pcl_pc2);
    pcl::PointCloud<RefPointType>::Ptr cloud_tf_pcl(new pcl::PointCloud<RefPointType>);
    pcl::fromPCLPointCloud2(pcl_pc2,*cloud_tf_pcl); 
    std::cerr << "cloud_tf_pcl : " << cloud_tf_pcl->width * cloud_tf_pcl->height << " data points." << std::endl;

    // light filter
    cloud_pass_.reset (new Cloud);
    cloud_pass_downsampled_.reset (new Cloud);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    filterPassThrough (cloud_tf_pcl, *cloud_pass_);

    if (counter_ < nb_wait_iter) // wait initialization
    {
      gridSample (cloud_pass_, *cloud_pass_downsampled_, downsampling_grid_size_);
    }
    else if (counter_ == nb_wait_iter) // set object to track
    {

      std::cerr << "ref_cloud: " << ref_cloud->width * ref_cloud->height << " data points." << std::endl;

      RefCloudPtr nonzero_ref (new RefCloud);
      removeZeroPoints (ref_cloud, *nonzero_ref); // OBJECT TO TRACK !!!!
      std::cerr << "nonzero_ref: " << nonzero_ref->width * nonzero_ref->height << " data points." << std::endl;
      
      PCL_INFO ("calculating cog\n");
      
      Eigen::Vector4f c;
      RefCloudPtr transed_ref (new RefCloud);
      pcl::compute3DCentroid<RefPointType> (*nonzero_ref, c); // obj init pos : centroid
      Eigen::Affine3f trans = Eigen::Affine3f::Identity ();
      trans.translation ().matrix () = Eigen::Vector3f (c[0], c[1], c[2]);
//      pcl::transformPointCloudWithNormals<RefPointType> (*ref_cloud, *transed_ref, trans.inverse());
      std::cout << "Matrix before " <<  trans.matrix() << std::endl;
      pcl::transformPointCloud<RefPointType> (*nonzero_ref, *transed_ref, trans.inverse()); // store it into trans
      std::cout << "Matrix after " <<  trans.matrix() << std::endl;

      CloudPtr transed_ref_downsampled (new Cloud); 
      gridSample (transed_ref, *transed_ref_downsampled, downsampling_grid_size_);
      tracker_->setReferenceCloud (transed_ref_downsampled); // TRACK THIS CLOUD !!
      tracker_->setTrans (trans);
      reference_ = transed_ref;
      tracker_->setMinIndices (int (ref_cloud->points.size ()) / 2); // SET INDICES TO TRACK

    }
    else //track the object
    {      
      std::cerr << "PointCloud before downsampled: " << cloud_pass_->width * cloud_pass_->height << " data points." << std::endl;
      gridSampleApprox (cloud_pass_, *cloud_pass_downsampled_, downsampling_grid_size_);
      std::cerr << "PointCloud after downsampled: " << cloud_pass_downsampled_->width * cloud_pass_downsampled_->height << " data points." << std::endl;
      tracker_->setInputCloud (cloud_pass_downsampled_);
      tracker_->compute ();
    }
    
    new_cloud_ = true;
    double end = pcl::getTime ();
    computation_time_ = end - start;
    FPS_CALC_END("computation");
    counter_++;

    // Call viewer callback
    viz_cb(viewer_);
  }
      

  void
  run (int argc, char** argv)
  { 
    std::cout << "RUN" << std::endl;

    // Initialize ROS
    ros::init (argc, argv, "create_model");
    ros::NodeHandle nh;

    // Load object to track (already in the base frame)
    ref_cloud.reset(new Cloud());
    if(pcl::io::loadPCDFile (argv[1], *ref_cloud) == -1){
      std::cout << "pcd file not found" << std::endl;
      exit(-1);
    }  

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("/kinect2/qhd/points", 1, &OpenNISegmentTracking::cloud_cb, this);
    ros::spin ();

    // while (!viewer_.wasStopped ())
    while (ros::ok())
      viewer_.spinOnce (100, true);
      boost::this_thread::sleep( boost::posix_time::milliseconds(100));
  }
  
  // pcl::visualization::CloudViewer viewer_;
  pcl::visualization::PCLVisualizer viewer_ ;
  CloudPtr cloud_pass_;
  CloudPtr cloud_pass_downsampled_;
  CloudPtr ref_cloud; // object cloud to track before translation
  CloudPtr reference_;
  pcl::PointCloud<RefPointType>::Ptr tracked_cloud_;
  
  std::string device_id_;
  boost::mutex mtx_;
  bool new_cloud_;
  pcl::NormalEstimationOMP<PointType, pcl::Normal> ne_; // to store threadpool
  boost::shared_ptr<ParticleFilter> tracker_;
  int counter_;
  bool use_convex_hull_;
  bool visualize_non_downsample_;
  bool visualize_particles_;
  double tracking_time_;
  double computation_time_;
  double downsampling_time_;
  double downsampling_grid_size_;
  };

int
main (int argc, char** argv)
{
  bool use_convex_hull = true;
  bool visualize_non_downsample = true;
  bool visualize_particles = false;
  bool use_fixed = false;

  double downsampling_grid_size = 0.01;
  
  std::string device_id = "#1";
  
  // open kinect
  OpenNISegmentTracking<pcl::PointXYZRGBA> v (device_id, 16, downsampling_grid_size,
                                              use_convex_hull,
                                              visualize_non_downsample, visualize_particles,
                                              use_fixed);
  v.run (argc, argv);
}
