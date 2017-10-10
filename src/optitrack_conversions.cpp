#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <pcl_tracking/ObjectPosition.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "object_base_frame_publisher_node");
    ros::NodeHandle n;

    ros::Publisher object_position_publisher = n.advertise<pcl_tracking::ObjectPosition>("/visual/obj_pos_vector", 1000);
    tf::StampedTransform transform;
    tf::TransformListener listener;
    geometry_msgs::PointStamped msg;
    pcl_tracking::ObjectPosition obj_pos_msg_;

    ros::Rate rate(200);

    while(ros::ok()){
        try {
            listener.lookupTransform("base", "optitrack/blue_cylinder", ros::Time(0), transform);
            //tf::transformStampedTFToMsg(transform, msg);
            msg.header.frame_id = transform.frame_id_;
            msg.header.stamp = transform.stamp_;
            msg.point.x = transform.getOrigin().getX();
            msg.point.y = transform.getOrigin().getY();
            msg.point.z = transform.getOrigin().getZ();

            obj_pos_msg_.object_position.push_back(msg);
        }
        catch (tf::TransformException &ex) {
            ROS_ERROR("%s",ex.what());
        }

        object_position_publisher.publish(obj_pos_msg_);
        obj_pos_msg_.object_position.clear();
        rate.sleep();
    }

    return 0;
}
