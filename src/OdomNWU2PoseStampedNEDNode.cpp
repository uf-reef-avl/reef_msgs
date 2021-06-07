//
// Created by prashant on 6/7/21.
//

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/tf.h>
#include <tf2_eigen/tf2_eigen.h>

static ros::Publisher poseStampedPub;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    geometry_msgs::PoseStamped poseStampedMsg;
    poseStampedMsg.header = msg->header;
    poseStampedMsg.pose.position.x = msg->pose.pose.position.x;
    poseStampedMsg.pose.position.y = -msg->pose.pose.position.y;
    poseStampedMsg.pose.position.z = -msg->pose.pose.position.z;
    poseStampedMsg.pose.orientation.x = msg->pose.pose.orientation.x;
    poseStampedMsg.pose.orientation.y = -msg->pose.pose.orientation.y;
    poseStampedMsg.pose.orientation.z = -msg->pose.pose.orientation.z;
    poseStampedMsg.pose.orientation.w = msg->pose.pose.orientation.w;

    poseStampedPub.publish(poseStampedMsg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Odom2PoseStamped");

    ros::NodeHandle n;

    ros::Subscriber odom_sub = n.subscribe("odom_nwu", 1000, odomCallback);
    poseStampedPub = n.advertise<geometry_msgs::PoseStamped>("pose_stamped_ned", 1000);

    ros::spin();

    return 0;
}
