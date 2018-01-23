#include <ros/ros.h>
#include <stdio.h>
#include <fstream>

#include "quaternion.h"
#include <yaml-cpp/yaml.h>

#include <Eigen/Core>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <pcl_tracking/ObjectPosition.h>
#include "mocap_optitrack/ObjectPositionID.h"
#include "mocap_optitrack/ObjectPoseID.h"

Eigen::Vector3d gTransToRobotBase;
Eigen::MatrixXd gRotToRobotBase(3,3);
bool transformed = false;
mocap_optitrack::ObjectPositionID msg;
std::vector< std::pair<int, Eigen::Vector3d> > objects_positions_vector;
//Eigen::Vector3d position_optitrack_frame, position_robot_frame;
//geometry_msgs::PointStamped msg;
//std::map<int, Eigen::Vector3d> objects_positions;

void transform_to_robotbase(const Eigen::Vector3d &pos_optitrack, Eigen::Vector3d &pos_robotbase)
{
    pos_robotbase = gRotToRobotBase*pos_optitrack + gTransToRobotBase;
    //pos_robotbase = gRotToRobotBase.transpose()*(pos_optitrack - gTransToRobotBase);
}

//void object_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& object_position)
void croissant_pose_callback(const mocap_optitrack::ObjectPositionID::ConstPtr& object_position)
{
    Eigen::Vector3d position_optitrack_frame, position_robot_frame;
    position_optitrack_frame(0) = object_position->object_position.point.x;
    position_optitrack_frame(1) = object_position->object_position.point.z;
    position_optitrack_frame(2) = -object_position->object_position.point.y;
    transform_to_robotbase(position_optitrack_frame, position_robot_frame);
    std::pair<int, Eigen::Vector3d> current_obj (object_position->ID, position_robot_frame);
    objects_positions_vector.push_back(current_obj);
    transformed = true;
}

void hook_pose_callback(const mocap_optitrack::ObjectPositionID::ConstPtr& object_position)
{
    Eigen::Vector3d position_optitrack_frame, position_robot_frame;
    position_optitrack_frame(0) = object_position->object_position.point.x;
    position_optitrack_frame(1) = object_position->object_position.point.z;
    position_optitrack_frame(2) = -object_position->object_position.point.y;
    transform_to_robotbase(position_optitrack_frame, position_robot_frame);
    std::pair<int, Eigen::Vector3d> current_obj (object_position->ID, position_robot_frame);
    objects_positions_vector.push_back(current_obj);
    transformed = true;
}

int load_tf(const std::string& tf_file)
{
    // below code works well on my computer but not in bxter-01 pc. need to be verified.
    //YAML::Node config = YAML::LoadFile(tf_file);
    //std::vector<double > pos  = config[0].as<std::vector<double > >();
    //std::vector<double > quat = config[1].as<std::vector<double > >();

    // simple workaround
    std::ifstream tf_file_stream(tf_file);
    std::string line;

    char str [20];
    Eigen::VectorXd pos(3), quat(4);

    std::getline(tf_file_stream, line);
    std::sscanf(line.c_str(), "%s [%lf, %lf, %lf]",str, &pos(0), &pos(1), &pos(2) );

    std::getline(tf_file_stream, line);
    std::sscanf(line.c_str(), "%s [%lf, %lf, %lf, %lf]",str, &quat(1), &quat(2), &quat(3), &quat(0));

    tf_file_stream.close();

    for(int i=0; i<3; i++) gTransToRobotBase(i) = pos(i);

    Quaternion q(quat(0), quat(1), quat(2), quat(3));
    q.normalize();

    gRotToRobotBase = q;
    std::cout << "tf pos : " << gTransToRobotBase << std::endl;
    std::cout << "tf rot : " << gRotToRobotBase << std::endl;
    std::cout << "q0 : " << quat(0) << " q1 : " << quat(1) << " q2 : " << quat(2) << " q3 : "  << quat(3) << std::endl;

    return 0;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "object_base_frame_publisher_node");
    ros::NodeHandle n;

    ros::Publisher object_position_publisher = n.advertise<pcl_tracking::ObjectPosition>("/visual/obj_pos_vector", 1000);
    //ros::Subscriber object_position_subscriber = n.subscribe<geometry_msgs::PoseStamped >("/optitrack/blue_cylinder/pose", 1, object_pose_callback);
    ros::Subscriber object_position_subscriber_croissant_pose_callback =
            n.subscribe<mocap_optitrack::ObjectPositionID >("/optitrack/croissant/position", 1, croissant_pose_callback);
    ros::Subscriber object_position_subscriber_hook =
            n.subscribe<mocap_optitrack::ObjectPositionID >("/optitrack/hook/position", 1, hook_pose_callback);

    pcl_tracking::ObjectPosition obj_pos_msg_;
    std::string tf_robotbase_file;
    n.getParam("tf_robotbase_file", tf_robotbase_file);
    load_tf(tf_robotbase_file);

    ros::Rate rate(20);

    while(ros::ok()){
        try {
            if(transformed){
                for (const std::pair<int, Eigen::Vector3d>& curr_obj : objects_positions_vector) {
                    msg.ID = curr_obj.first;
                    msg.object_position.point.x = curr_obj.second(0);
                    msg.object_position.point.y = curr_obj.second(1);
                    msg.object_position.point.z = curr_obj.second(2);
                    obj_pos_msg_.object_position.push_back(msg);
                }
            }
        }
        catch (tf::TransformException &ex) {
            ROS_ERROR("%s",ex.what());
        }
        object_position_publisher.publish(obj_pos_msg_);

        obj_pos_msg_.object_position.clear(); // Clean message
        objects_positions_vector.clear(); // Clean vector of positions gathered
        transformed = false;
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
