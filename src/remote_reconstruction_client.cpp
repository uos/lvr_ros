#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>

#include "move_base_flex/navigation_utility.h"

#include "lvr_ros/SendCloudAction.h"
#include "lvr_ros/StartReconstructionAction.h"

ros::Subscriber cloud_sub;
boost::shared_ptr<actionlib::SimpleActionClient<lvr_ros::SendCloudAction> > client_send_ptr;
boost::shared_ptr<actionlib::SimpleActionClient<lvr_ros::StartReconstructionAction> > client_reconstruct_ptr;

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
    ROS_INFO_STREAM("Received cloud.");
    lvr_ros::SendCloudGoal goal;
    goal.cloud = *cloud;
    client_send_ptr->sendGoal(goal);

    geometry_msgs::PoseStamped robot_pose;
    tf::TransformListener tf_listener;
    const std::string robot_frame = "riegl_meas_origin";
    const std::string global_frame = "odom_combined";
    move_base_flex::getRobotPose(tf_listener, robot_frame, global_frame, ros::Duration(2), robot_pose);
    goal.pose = robot_pose;

    ROS_INFO_STREAM("Sending cloud...");
    bool result = client_send_ptr->waitForResult(ros::Duration(5));
    if (not result)
    {
        ROS_ERROR_STREAM("Cloud not sent.");
    } else
    {
        ROS_INFO_STREAM("Cloud sent successfully.");
    }

}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "remote_reconstruction_client");
    ros::NodeHandle nh("~");
    client_send_ptr = boost::shared_ptr<actionlib::SimpleActionClient<lvr_ros::SendCloudAction> >
        (new actionlib::SimpleActionClient<lvr_ros::SendCloudAction>("/send"));
    client_reconstruct_ptr = boost::shared_ptr<actionlib::SimpleActionClient<lvr_ros::StartReconstructionAction> >
        (new actionlib::SimpleActionClient<lvr_ros::StartReconstructionAction>("/reconstruct"));
    ROS_INFO_STREAM("Started cloud client.");
    cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("/riegl_cloud", 1, pointCloudCallback);
    client_send_ptr->waitForServer();
    client_reconstruct_ptr->waitForServer();
    ros::spin();
    return 0;
}
