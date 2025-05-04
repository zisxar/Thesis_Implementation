#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <tf2/convert.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>

#include <pole/SetLength.h>

using namespace std;

bool length_set = false;
float length = 0.0f;

bool set_length_handler(pole::SetLength::Request  &req, pole::SetLength::Response &res) {
    ROS_INFO("length= %f", (float)req.length);
    length = req.length;
    length_set = true;
    res.set_ok = true;
    return true;
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "nail_publisher");

    ros::NodeHandle nh("~");

    float default_length;
    if (nh.getParam("default_length", default_length)) {
      ROS_INFO("Default length is: %f", default_length);
      length = default_length;
      length_set = true;
    }
    else {
      ROS_WARN("Failed to get param 'default_length'");
      length = 0.0f;
      length_set = false;
    }

    ros::ServiceServer length_service = nh.advertiseService("/nail_publisher/set_length", set_length_handler);

    ros::Publisher coordinates_pub = nh.advertise<geometry_msgs::PointStamped>("/nail_publisher/coordinates", 10);

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    // tf2_ros::TransformBroadcaster tf_br;
    // geometry_msgs::TransformStamped transform_rs_nail;
    // geometry_msgs::TransformStamped transform_rs_nail2;
    geometry_msgs::TransformStamped transform_map_rs;
    geometry_msgs::PointStamped point_nail;

    tf2::Transform tf_map_rs;
    tf2::Transform tf_rs_nail;
    tf2::Transform tf_map_nail;

    tf2::Vector3 nail_coords;
    // tf2::Vector3 nail_trans;
    // tf2::Quaternion nail_rot;

    ros::Rate rate(30.0);

    while (nh.ok()){
        // ROS_INFO("LOOP");

        try{
            if (length_set) {

                // Find the transform map -> realsense
                transform_map_rs = tfBuffer.lookupTransform("map", "realsense", ros::Time(0));
                //ROS_INFO("x= %f, y= %f, z= %f", (float)transform_map_rs.transform.translation.x, (float)transform_map_rs.transform.translation.y, (float)transform_map_rs.transform.translation.z);

                tf_map_rs.setOrigin(tf2::Vector3(transform_map_rs.transform.translation.x, transform_map_rs.transform.translation.y, transform_map_rs.transform.translation.z));
                tf_map_rs.setRotation(tf2::Quaternion(transform_map_rs.transform.rotation.x, transform_map_rs.transform.rotation.y, transform_map_rs.transform.rotation.z, transform_map_rs.transform.rotation.w));

                // Assign the transform realsense -> nail
                tf_rs_nail.setOrigin(tf2::Vector3(0.0, -0.085, -length));
                tf_rs_nail.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));

                // Find the transform map -> nail

                tf_map_nail = tf_map_rs * tf_rs_nail;

                // Assign the coordinates
                nail_coords = tf_map_nail.getOrigin();

                point_nail.header = transform_map_rs.header;
                point_nail.point.x = nail_coords.x();
                point_nail.point.y = nail_coords.y();
                point_nail.point.z = nail_coords.z();

                // Publish coordinates
                coordinates_pub.publish(point_nail);

                // transform_rs_nail2.header.stamp = ros::Time::now();
                // transform_rs_nail2.header.frame_id = "map";
                // transform_rs_nail2.child_frame_id = "nail_2";
                // transform_rs_nail2.transform.translation.x = point_nail.x;
                // transform_rs_nail2.transform.translation.y = point_nail.y;
                // transform_rs_nail2.transform.translation.z = point_nail.z;
                // transform_rs_nail2.transform.rotation.x = 0.0;
                // transform_rs_nail2.transform.rotation.y = 0.0;
                // transform_rs_nail2.transform.rotation.z = 0.0;
                // transform_rs_nail2.transform.rotation.w = 1.0;

                // tf_br.sendTransform(transform_rs_nail2);

                // Publish transform realsene -> nail

                // nail_trans = tf_rs_nail.getOrigin();
                // nail_rot = tf_rs_nail.getRotation();

                // transform_rs_nail.header.stamp = ros::Time::now();
                // transform_rs_nail.header.frame_id = "realsense";
                // transform_rs_nail.child_frame_id = "nail";
                // transform_rs_nail.transform.translation.x = nail_trans.x();
                // transform_rs_nail.transform.translation.y = nail_trans.y();
                // transform_rs_nail.transform.translation.z = nail_trans.z();
                // transform_rs_nail.transform.rotation.x = nail_rot.x();
                // transform_rs_nail.transform.rotation.y = nail_rot.y();
                // transform_rs_nail.transform.rotation.z = nail_rot.z();
                // transform_rs_nail.transform.rotation.w = nail_rot.w();

                // tf_br.sendTransform(transform_rs_nail);
            }
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
