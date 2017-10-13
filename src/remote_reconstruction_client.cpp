#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>

#include "move_base_flex/navigation_utility.h"

#include "lvr_ros/SendCloudAction.h"
#include "lvr_ros/StartReconstructionAction.h"

using namespace lvr_ros;

class CloudClient
{
    private:
        ros::Subscriber cloud_sub;
        /// action clients
        actionlib::SimpleActionClient<SendCloudAction> client_send;
        actionlib::SimpleActionClient<StartReconstructionAction> client_reconstruct;
        int n_clouds;

        void sendTrigger()
        {
            StartReconstructionGoal goal;
            client_reconstruct.sendGoal(goal);
            bool result = client_reconstruct.waitForResult(ros::Duration(5));
            if (not result)
            {
                ROS_ERROR_STREAM("Trigger not sent.");
                n_clouds = 0;
            } else
            {
                ROS_INFO_STREAM("Trigger sent successfully.");
            }
        }

        void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud)
        {
            ROS_INFO_STREAM("Received cloud.");
            SendCloudGoal goal;
            goal.cloud = *cloud;
            client_send.sendGoal(goal);

            geometry_msgs::PoseStamped robot_pose;
            tf::TransformListener tf_listener;
            const std::string robot_frame = "riegl_meas_origin";
            const std::string global_frame = "odom_combined";
            move_base_flex::getRobotPose(tf_listener, robot_frame, global_frame, ros::Duration(2), robot_pose);
            goal.pose = robot_pose;

            ROS_INFO_STREAM("Sending cloud...");
            bool result = client_send.waitForResult(ros::Duration(5));
            if (not result)
            {
                ROS_ERROR_STREAM("Cloud not sent.");
            } else
            {
                ++n_clouds;
                ROS_INFO_STREAM("Cloud sent successfully.");
            }

            if (n_clouds >= 1)
            {
                sendTrigger();
            }

        }

    public:
        CloudClient(ros::NodeHandle& nh) : client_send("/send"), client_reconstruct("/reconstruct"), n_clouds(0)
    {
        cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("/riegl_cloud", 1,
                &CloudClient::pointCloudCallback, this);
        ROS_INFO_STREAM("Waiting for servers.");
        client_send.waitForServer();
        client_reconstruct.waitForServer();
    }
};


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "remote_reconstruction_client");
    ros::NodeHandle nh("~");
    static CloudClient client(nh);
    ROS_INFO_STREAM("Started cloud client.");
    ros::spin();
    return 0;
}
