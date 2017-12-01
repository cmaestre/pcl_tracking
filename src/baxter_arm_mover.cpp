#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>

int main(int argc, char **argv){
        ros::init(argc, argv, "baxter_arm_mover_node");
        ros::NodeHandle nh;

        double start_x, start_y, start_z, stop_x, stop_y, stop_z;

        moveit::planning_interface::MoveGroup group("left_arm");

        ros::spinOnce();

        ros::Rate rate(1);

        while(ros::ok()){

                ROS_INFO_STREAM("TEST: Robot left arm position X : " << group.getCurrentPose().pose.position.x
                                << "  Y : " << group.getCurrentPose().pose.position.y
                                << "  Z : " << group.getCurrentPose().pose.position.z);

                ROS_INFO("Please Enter start point ...");

                std::cin >> start_x >> start_y >> start_z;

                ROS_INFO("Please Enter stop point ...");

                std::cin >> stop_x >> stop_y >> stop_z;

                std::vector<geometry_msgs::Pose> waypoints;
                //waypoints.push_back(group.getCurrentPose().pose);
                geometry_msgs::Pose start_pose;
                start_pose.position.x = start_x;
                start_pose.position.y = start_y;
                start_pose.position.z = start_z;
//                start_pose.orientation.w = group.getCurrentPose().pose.orientation.w;
//                start_pose.orientation.x = group.getCurrentPose().pose.orientation.x;
//                start_pose.orientation.y = group.getCurrentPose().pose.orientation.y;
//                start_pose.orientation.z = group.getCurrentPose().pose.orientation.z;
                start_pose.orientation.w = 0.0;
                start_pose.orientation.x = 0.0;
                start_pose.orientation.y = 1.0;
                start_pose.orientation.z = 0.0;
                geometry_msgs::Pose stop_pose;
                stop_pose.position.x = stop_x;
                stop_pose.position.y = stop_y;
                stop_pose.position.z = stop_z;
//                stop_pose.orientation.w = group.getCurrentPose().pose.orientation.w;
//                stop_pose.orientation.x = group.getCurrentPose().pose.orientation.x;
//                stop_pose.orientation.y = group.getCurrentPose().pose.orientation.y;
//                stop_pose.orientation.z = group.getCurrentPose().pose.orientation.z;
                stop_pose.orientation.w = 0.0;
                stop_pose.orientation.x = 0.0;
                stop_pose.orientation.y = 1.0;
                stop_pose.orientation.z = 0.0;



                waypoints.push_back(start_pose);
                waypoints.push_back(stop_pose);

                moveit_msgs::RobotTrajectory robot_trajectory;
                double fraction = group.computeCartesianPath(waypoints, 0.01, 0.0, robot_trajectory);
                if(fraction == 1){
                        moveit::planning_interface::MoveGroup::Plan the_plan;
                        the_plan.trajectory_ = robot_trajectory;
                        group.execute(the_plan);
                    }

                ros::spinOnce();
                rate.sleep();
            }

        return 0;
    }

