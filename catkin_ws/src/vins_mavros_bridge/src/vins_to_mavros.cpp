#include <cmath>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

ros::Publisher pub;

namespace
{
geometry_msgs::Quaternion normalize_quaternion(
    double x, double y, double z, double w)
{
    geometry_msgs::Quaternion q;
    const double norm = std::sqrt(x * x + y * y + z * z + w * w);
    if (norm <= 1e-12) {
        q.x = 0.0;
        q.y = 0.0;
        q.z = 0.0;
        q.w = 1.0;
        return q;
    }

    q.x = x / norm;
    q.y = y / norm;
    q.z = z / norm;
    q.w = w / norm;
    return q;
}

geometry_msgs::Quaternion multiply_quaternion(
    const geometry_msgs::Quaternion& a,
    const geometry_msgs::Quaternion& b)
{
    return normalize_quaternion(
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z);
}
}  // namespace

void vins_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = msg->header.stamp;
    pose.header.frame_id = "map";

    pose.pose.position.x =  msg->pose.pose.position.y;
    pose.pose.position.y = -msg->pose.pose.position.x;
    pose.pose.position.z =  msg->pose.pose.position.z;

    // Temporary A/B test: bypass attitude axis remap and fixed body rotation
    // compensation, and publish the raw VINS quaternion directly.
    // double qx = msg->pose.pose.orientation.x;
    // double qy = msg->pose.pose.orientation.y;
    // double qz = msg->pose.pose.orientation.z;
    // double qw = msg->pose.pose.orientation.w;
    //
    // geometry_msgs::Quaternion map_optical_q;
    // map_optical_q.x =  qy;
    // map_optical_q.y = -qx;
    // map_optical_q.z =  qz;
    // map_optical_q.w =  qw;
    //
    // geometry_msgs::Quaternion optical_to_base_q;
    // optical_to_base_q.x = 0.0;
    // optical_to_base_q.y = -std::sqrt(0.5);
    // optical_to_base_q.z = 0.0;
    // optical_to_base_q.w =  std::sqrt(0.5);
    //
    // pose.pose.orientation = multiply_quaternion(
    //     map_optical_q, optical_to_base_q);
    pose.pose.orientation = msg->pose.pose.orientation;

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
        "/vins_estimator/imu_propagate", 100, vins_cb);

    publish_static_tf();

    ros::spin();
    return 0;
}
