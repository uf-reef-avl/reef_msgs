//
// Created by prashant on 6/7/21.
//

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"


class NWUOdom2NEDPoseStamped: public rclcpp::Node {
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr poseStampedPub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub;
    public:
    NWUOdom2NEDPoseStamped(): Node("Odom2PoseStamped")
        {
            poseStampedPub = this->create_publisher<geometry_msgs::msg::PoseStamped>("~/pose_stamped_ned",100);
//            odomSub = this->create_subscription<nav_msgs::msg::Odometry>("~/odometry_nwu", 10, std::bind(&NWUOdom2NEDPoseStamped::odomCallback, this, _1));
            odomSub = this->create_subscription<nav_msgs::msg::Odometry>("~/odometry_nwu", 10, std::bind(&NWUOdom2NEDPoseStamped::odomCallback, this, std::placeholders::_1));
        }

        void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        geometry_msgs::msg::PoseStamped poseStampedMsg;
        poseStampedMsg.header = msg->header;
        poseStampedMsg.pose.position.x = msg->pose.pose.position.x;
        poseStampedMsg.pose.position.y = -msg->pose.pose.position.y;
        poseStampedMsg.pose.position.z = -msg->pose.pose.position.z;
        poseStampedMsg.pose.orientation.x = msg->pose.pose.orientation.x;
        poseStampedMsg.pose.orientation.y = -msg->pose.pose.orientation.y;
        poseStampedMsg.pose.orientation.z = -msg->pose.pose.orientation.z;
        poseStampedMsg.pose.orientation.w = msg->pose.pose.orientation.w;

        poseStampedPub->publish(poseStampedMsg);
    }
};






int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NWUOdom2NEDPoseStamped>());
    rclcpp::shutdown();
    return 0;


    return 0;
}
