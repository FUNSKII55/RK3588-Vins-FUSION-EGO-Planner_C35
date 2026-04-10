#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

ros::Publisher pub;

void vins_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = msg->header.stamp;
    pose.header.frame_id = "map";

    pose.pose.position.x =  msg->pose.pose.position.y;
    pose.pose.position.y = -msg->pose.pose.position.x;
    pose.pose.position.z =  msg->pose.pose.position.z;

    double qx = msg->pose.pose.orientation.x;
    double qy = msg->pose.pose.orientation.y;
    double qz = msg->pose.pose.orientation.z;
    double qw = msg->pose.pose.orientation.w;

    pose.pose.orientation.x =  qy;
    pose.pose.orientation.y = -qx;
    pose.pose.orientation.z =  qz;
    pose.pose.orientation.w =  qw;

    pub.publish(pose);
}

void publish_static_tf()
{
    static tf2_ros::StaticTransformBroadcaster br;
    geometry_msgs::TransformStamped tf;

    tf.header.stamp = ros::Time::now();
    tf.header.frame_id = "base_link";
    tf.child_frame_id = "camera_link";

    // 相机在飞控前方 9cm、上方 4cm、无左右偏移
    // ROS FLU 坐标系：X前 Y左 Z上
    tf.transform.translation.x = 0.09;
    tf.transform.translation.y = 0.0;
    tf.transform.translation.z = 0.04;

    // 相机朝前，yaw=0，无旋转
    tf.transform.rotation.x = 0.0;
    tf.transform.rotation.y = 0.0;
    tf.transform.rotation.z = 0.0;
    tf.transform.rotation.w = 1.0;

    br.sendTransform(tf);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vins_to_mavros");
    ros::NodeHandle nh;

    pub = nh.advertise<geometry_msgs::PoseStamped>(
        "/mavros/vision_pose/pose", 10);

    ros::Subscriber sub = nh.subscribe(
        "/vins_estimator/odometry", 10, vins_cb);

    publish_static_tf();

    ros::spin();
    return 0;
}
