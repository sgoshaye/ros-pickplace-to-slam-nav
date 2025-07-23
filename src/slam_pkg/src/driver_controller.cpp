#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>

tf::Transform transform;
tf::Quaternion q;

void pose_callback(const geometry_msgs::PoseStampedPtr &pose) {
    static tf::TransformBroadcaster br;
    q.setX(pose->pose.orientation.x);
    q.setY(pose->pose.orientation.y);
    q.setZ(pose->pose.orientation.z);
    q.setW(pose->pose.orientation.w);

    transform.setOrigin(tf::Vector3(pose->pose.position.x, pose->pose.position.y, 0.0));
    transform.setRotation(q);

    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));
}

void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& cmd_vel) {
    // Replace this with your code to command the motor controller
    ROS_INFO("Received cmd_vel - Linear: [%f, %f], Angular: [%f]",
             cmd_vel->linear.x, cmd_vel->linear.y, cmd_vel->angular.z);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "driver_controller");
    ros::NodeHandle n;

    ros::Subscriber pose_sub = n.subscribe("pose", 1, pose_callback);
    ros::Subscriber cmd_vel_sub = n.subscribe("cmd_vel", 1, cmd_vel_callback);

    ros::Rate loop_rate(100);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
}

