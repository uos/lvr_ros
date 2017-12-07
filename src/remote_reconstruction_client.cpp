#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <thread>
#include <cstring>


#include <tf/transform_listener.h>
#include <lvr_ros/slam6d_ros_utils.hpp>


#include <lvr_ros/SendCloudAction.h>
#include <lvr_ros/StartReconstructionAction.h>
#include <lvr_ros/StopReconstructionAction.h>

#include <move_base_flex/navigation_utility.h>
using namespace lvr_ros;

typedef actionlib::SimpleActionClient<SendCloudAction> SendClient;
typedef actionlib::SimpleActionClient<StartReconstructionAction> ReconstructClient;
typedef actionlib::SimpleActionClient<StopReconstructionAction> StopReconstructClient;

class CloudClient
{
    private:
        ros::Subscriber cloud_sub;
        /// action clients
        SendClient client_send;
        ReconstructClient client_reconstruct;
        StopReconstructClient client_stop;
        int n_clouds;
        int max_clouds;
        tf::TransformListener transform_listener;

        void sendStop()
        {
            // I have tested this by running a thread which will call this
            // method 5 secs after trigger and it worked. It has to be done
            // asynchronously though, since sendTrigger() block by waiting for
            // the action server result,
            StopReconstructionGoal goal;
            client_stop.sendGoal(goal);
            bool result = client_stop.waitForResult();
            if (not result)
            {
                ROS_ERROR_STREAM("Stop not sent.");
            } else
            {
                ROS_INFO_STREAM("Stop sent successfully.");
            }
            n_clouds = 0;
        }

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

            std::vector<double> rP, rPT;
            bool success = getTransform(rP, rPT, transform_listener,
                    ros::Time::now(), "riegl_meas_origin", "odom_combined");
            goal.rP = rP;
            goal.rPT = rPT;

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
        CloudClient(ros::NodeHandle& nh) :
            client_send("/send"),
            client_reconstruct("/reconstruct"),
            client_stop("/stop"),
            n_clouds(0),
            transform_listener(ros::Duration(60.0))
    {
        cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("/riegl_cloud", 5,
                &CloudClient::pointCloudCallback, this);
        ROS_INFO_STREAM("Waiting for servers.");
        if (not nh.hasParam("numClouds"))
        {
            ROS_ERROR("Don't know how many clouds to expect.");
            exit(1);
        } else
        {
            nh.getParam("numClouds", max_clouds);
            ROS_INFO_STREAM("Will stop after " << max_clouds << " clouds.");
            client_send.waitForServer();
            client_reconstruct.waitForServer();
            client_stop.waitForServer();
        }
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
