#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2_msgs/TFMessage.h>
#include <pcl_tracking/ObjectPosition.h>

class tf_publisher{
public:
    void init(){
        _stamped_transform_publisher = _nh.advertise<geometry_msgs::TransformStamped>("/visual/object_position_base_frame", 1000);
        _object_update_sub = _nh.subscribe<pcl_tracking::ObjectPosition>("/visual/cam_frame_obj_pos_vector", 10, &tf_publisher::object_callback, this);

        ros::AsyncSpinner my_spinner(4);
        my_spinner.start();
    }

    void publish_point_frame(geometry_msgs::PointStamped point, std::string child_frame_id){
        // publish transform
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(point.point.x,
                                         point.point.y,
                                         point.point.z));

        tf::Quaternion q(0.,
                         0.,
                         0.,
                         1.0);

        // Handle different coordinate systems (Arena vs. rviz)
        transform.setRotation(q);
        ros::Time timestamp(ros::Time::now());
        tf_pub.sendTransform(tf::StampedTransform(transform, timestamp, "kinect2_link", child_frame_id));
    }

    void object_callback(const pcl_tracking::ObjectPositionConstPtr& object_msgs){
        _object_size = object_msgs->object_position.size();
        for(size_t i = 0; i < _object_size; i++){


        //ROS_WARN_STREAM("The choosen point in camera frame is: X = " << point.point.x << ", Y = " << point.point.y << ", and Z = " << point.point.z);


        publish_point_frame(object_msgs->object_position[i], "/visual/object_base_frame_" + std::to_string(i));
        }
    }

    int get_object_size(){
        return _object_size;
    }

private:
    ros::NodeHandle _nh;
    ros::Publisher _stamped_transform_publisher;
    ros::Subscriber _object_update_sub;
    tf::TransformBroadcaster tf_pub;
    tf::StampedTransform stamped_transform;
    geometry_msgs::TransformStamped msg;
    int _object_size;
};



int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_octomap_node");
    ros::NodeHandle node;

    ros::Publisher _objects_positions_pub = node.advertise<pcl_tracking::ObjectPosition>("/visual/obj_pos_vector", 1000);
    tf::StampedTransform transform;
    tf::TransformListener listener;
    geometry_msgs::TransformStamped msg;
    std::string parent_frame = "/base";
    std::string child_frame_id;
    pcl_tracking::ObjectPosition obj_pos_msg_;

    tf_publisher my_tf_publisher;
    my_tf_publisher.init();

    ros::Rate my_rate(20);
    while (ros::ok()) {
        ros::spinOnce();

        geometry_msgs::PointStamped point;
        for(int i = 0; i < my_tf_publisher.get_object_size(); i++){
            try{
                child_frame_id = "/visual/object_base_frame_" + std::to_string(i);
                listener.lookupTransform(parent_frame, child_frame_id, ros::Time(0), transform);
                tf::transformStampedTFToMsg(transform, msg);

                point.header.stamp = ros::Time::now();
                point.header.frame_id = parent_frame;
                point.point.x = msg.transform.translation.x;
                point.point.y = msg.transform.translation.y;
                point.point.z = msg.transform.translation.z;
                point.header.seq = i;
                obj_pos_msg_.object_position.push_back(point);
                ROS_ERROR_STREAM("Object " << i << " : " <<
                                                point.point.x << " " <<
                                                point.point.y << " " <<
                                                point.point.z);
            }
            catch (tf::TransformException &ex) {
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
            }
        }
        _objects_positions_pub.publish(obj_pos_msg_);
        obj_pos_msg_.object_position.clear();
        my_rate.sleep();
    }
    ros::waitForShutdown();
    return 0;
}
