#pragma once

#include "lvr_ros/ReconstructionConfig.h"
#include "lvr_ros/SendCloudAction.h"
#include "lvr_ros/StartReconstructionAction.h"
#include "lvr_ros/StopReconstructionAction.h"

#include <pcl_definitions/types/types.hpp>
#include <pcl_definitions/utils.hpp>
#include <condition_variable>
#include <thread>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <string>
#include <actionlib/server/simple_action_server.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <mesh_msgs/MeshGeometry.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <boost/filesystem/path.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

#include <Eigen/Geometry>
#include <pcl_ros/transforms.h>

#include "lvr_ros/slam6d_ros_utils.hpp"

namespace lvr_ros
{

    using namespace std;
    using namespace ros;
    using namespace pcl;
    namespace bfs = boost::filesystem;

    typedef dynamic_reconfigure::Server<lvr_ros::ReconstructionConfig>
        DynReconfigureServer;
    typedef boost::shared_ptr<DynReconfigureServer>
        DynReconfigureServerPtr;
    typedef actionlib::SimpleActionServer<lvr_ros::SendCloudAction>
        SendCloudActionServer;
    typedef actionlib::SimpleActionServer<lvr_ros::StartReconstructionAction>
        StartReconstructionActionServer;
    typedef actionlib::SimpleActionServer<lvr_ros::StopReconstructionAction>
        StopReconstructionActionServer;

    static const bfs::path remote_box_directory = "/tmp/clouds_remote";
    static const bfs::path local_box_directory  = "/tmp/clouds_local";
    static const bfs::path trigger_fname        = ".start_reconstruction";
    static const bfs::path pose_fname           = "pose.pose";
    static const bfs::path config_fname         = "remote_reconstruction_config.yaml";
    static const bfs::path stop_fname           = ".stop_reconstruction";
    static const bfs::path ready_fname          = ".done";
    static const bfs::path mesh_fname           = "triangle_mesh.ply";


    class RemoteReconstruction
    {
        public:
            RemoteReconstruction(const string remote_host = "localhost");

        private:
            condition_variable cv;  ///< condition variable for notifying main thread
            bool was_stopped;       ///< recrod whether notify was called due to preemption or finished
            mutex cv_m;             ///< mutex for notifying
            FileObserver observer;  ///< observer checking for events on local box dir
            string remote_host;     ///< host to copy files to
            DynReconfigureServerPtr reconfigure_server_ptr;
            DynReconfigureServer::CallbackType callback_type;

            ros::NodeHandle node_handle;
            ros::Publisher mesh_publisher;
            ReconstructionConfig config;
            SendCloudActionServer send_as;
            StartReconstructionActionServer reconstruct_as;
            StopReconstructionActionServer stop_as;
            tf::TransformListener transform_listener;
            int n_clouds;

            /**
             * @brief Save all dyn_conf parameters. NOTE: When adding/removing
             * params, you must adjust this function, since there is no way to
             * simply dump all parameters (you never know the type and
             * dynamic_reconfigure does not provide documentation)
             * This needs to be done for scp'ing it.
             *
             * @return Success or failure
             */
            bool writeCurrentConfig();

            /**
             * @brief Write a PointCloud2 to ply
             * This needs to be done for scp'ing it.
             * @param tmp_fname Filename to write to (in our use case only
             *                  temporary)
             * @param cloud Cloud to write
             * @return Sucess or failure
             */
            bool writePLY(const string& tmp_fname, const sensor_msgs::PointCloud2& cloud) const;

            /**
             * @brief Look up and write the current scanner pose. The scanner
             * frame ist fixed to "riegl_meas_origin", the robot/global frame to
             * "odom_combined"
             * This needs to be done for scp'ing it.
             * @param stamp Timestamp to use for querying tf
             * @return Success or failure
             */
            bool writePose(ros::Time stamp);

            /**
             * @brief Write trigger file for scp'ing
             * This needs to be done for scp'ing it.
             * @return Success or failure
             */
            bool writeTriggerFile();

            /**
             * @brief Write stop file for scp'ing
             * This needs to be done for scp'ing it.
             * @return Success or failure
             */
            bool writeStopFile();

            /**
             * @brief Reconfigure the node's parameters
             * @param config
             * @param level
             */
            void reconfigureCallback(lvr_ros::ReconstructionConfig& config, uint32_t level);

            /**
             * @brief Action callback for sending a cloud which will then be
             * scp'ed to the LVR server
             *
             * @param goal
             */
            void sendCloud(const lvr_ros::SendCloudGoalConstPtr& goal);

            /**
             * @brief Action callback for stopping an ongoing reconstruction.
             * This is achieved by copying a special file to the server via scp
             * which the server will see and interrupt processing. There is no
             * feedback whether this was successful, we just assume that it was.
             *
             * @param goal
             */
            void stopReconstruction(const lvr_ros::StopReconstructionGoalConstPtr& goal);

            /**
             * @brief Action callback for starting a reconstruction.
             * This is achieved by copying a special file to the server via scp
             * which the server will see and collect all present clouds and
             * start the pipeline. Once the server is done, it wil scp back a
             * special file and the result. We see the special file and mark the
             * action as succeeded.
             *
             * @param goal
             */
            void startReconstruction(const lvr_ros::StartReconstructionGoalConstPtr& goal);

            /**
             * @brief Things to perform when anything happens in the local
             * box directory.
             *
             * @param event
             */
            void onAnyEvent(const FSEvent& event);

            /**
             * @brief FileObserver callback for files changed.
             *
             * @param event
             */
            void fileModified(const FSEvent& event);

             /**
             * @brief FileObserver callback for files created,
             *
             * @param event
             */
            void fileCreated(const FSEvent& event);

             /**
             * @brief FileObserver callback for files deleted.
             *
             * @param event
             */
            void fileDeleted(const FSEvent& event);

            /**
             * @brief Publish a message from thre received lvr output
             * @param filename Filename of the triangle mesh PLY
             */
            mesh_msgs::MeshGeometryPtr meshFromFile(const std::string filename);
    };
}

