#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>

#include "move_base_flex/navigation_utility.h"

#include "lvr_ros/SendCloudAction.h"
#include "lvr_ros/StartReconstructionAction.h"

using namespace lvr_ros;

typedef actionlib::SimpleActionClient<SendCloudAction> SendClient;
typedef actionlib::SimpleActionClient<StartReconstructionAction> ReconstructClient;

class CloudClient
{
    private:
        ros::Subscriber cloud_sub;
        /// action clients
        SendClient client_send;
        ReconstructClient client_reconstruct;
        int n_clouds;
        int max_clouds;

        void sendTrigger()
        {
            StartReconstructionGoal goal;
            client_reconstruct.sendGoal(goal);
            bool result = client_reconstruct.waitForResult();
            if (not result)
            {
                ROS_ERROR_STREAM("Trigger not sent.");
            } else
            {
                ROS_INFO_STREAM("Trigger sent successfully.");
            }
            n_clouds = 0;
        }

        void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud)
        {
            ROS_INFO_STREAM("Received cloud.");
            SendCloudGoal goal;
            goal.cloud = *cloud;

            const std::string robot_frame = "riegl_meas_origin";
            const std::string global_frame = "odom_combined";
            client_send.sendGoal(goal);

            ROS_INFO_STREAM("Sending cloud...");
            bool result = client_send.waitForResult();
            if (not result)
            {
                ROS_ERROR_STREAM("Cloud not sent.");
            } else
            {
                ++n_clouds;
                ROS_INFO_STREAM("Cloud sent successfully.");
            }

            if (n_clouds >= max_clouds)
            {
                sendTrigger();
            }

        }

    public:
        CloudClient(ros::NodeHandle& nh) : client_send("/send"),
        client_reconstruct("/reconstruct"), n_clouds(0)
    {
        cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("/riegl_cloud", 5,
                &CloudClient::pointCloudCallback, this);
        ROS_INFO_STREAM("Waiting for servers.");
        nh.getParam("numClouds", max_clouds);
        ROS_INFO_STREAM("Will stop after " << max_clouds << " clouds.");
        client_send.waitForServer();
        client_reconstruct.waitForServer();
    }
};


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "remote_reconstruction_client");
    ros::NodeHandle nh("~");
    static CloudClient client(nh);
    ROS_INFO_STREAM("Started cloud client.");
    ros::spin();
    return 0;
}
