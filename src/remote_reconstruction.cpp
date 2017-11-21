#include "lvr_ros/ReconstructionConfig.h"
#include "lvr_ros/SendCloudAction.h"
#include "lvr_ros/StartReconstructionAction.h"
#include "lvr_ros/StopReconstructionAction.h"
#include "lvr_ros/FileObserver.hpp"

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
#include <mesh_msgs/TriangleMesh.h>
#include <mesh_msgs/TriangleMeshStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <pcl/io/ply_io.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <boost/filesystem/path.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

#include <Eigen/Geometry>
#include <pcl_ros/transforms.h>

#include "lvr_ros/slam6d_ros_utils.hpp"

// ansi escape for white on black
#define CMD_COLOR(stuff) ("\033[37;40m") << (stuff) << ("\033[0m")

namespace pt = boost::property_tree;

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


    class RemoteReconstruction
    {
        public:
            RemoteReconstruction(const string remote_host = "localhost") :
                send_as(node_handle,
                        "send",
                        boost::bind(&RemoteReconstruction::sendCloud, this, _1), false),
                reconstruct_as(node_handle,
                               "reconstruct",
                                boost::bind(&RemoteReconstruction::startReconstruction, this, _1),
                                false),
                stop_as(node_handle,
                        "stop",
                        boost::bind(&RemoteReconstruction::stopReconstruction, this, _1),
                        false),
                transform_listener(ros::Duration(60.0)),
                observer(local_box_directory),
                remote_host(remote_host),
                was_stopped(false)
        {

            mesh_publisher = node_handle.advertise<mesh_msgs::TriangleMeshStamped>("/mesh", 1);

            // setup dynamic reconfigure
            reconfigure_server_ptr = DynReconfigureServerPtr(new DynReconfigureServer(node_handle));
            callback_type = boost::bind(&RemoteReconstruction::reconfigureCallback, this, _1, _2);
            reconfigure_server_ptr->setCallback(callback_type);

            send_as.start();
            reconstruct_as.start();
            stop_as.start();

            if (not bfs::exists(remote_box_directory))
            {
                bfs::create_directory(remote_box_directory);
            }
            n_clouds = 0;

            this->observer.on_modified(bind(&RemoteReconstruction::fileModified,
                        this, placeholders::_1));
            this->observer.on_deleted(bind(&RemoteReconstruction::fileDeleted,
                        this, placeholders::_1));
            this->observer.on_created(bind(&RemoteReconstruction::fileCreated,
                        this, placeholders::_1));
        }

        private:
            condition_variable cv;
            bool was_stopped;
            mutex cv_m;
            FileObserver observer;
            string remote_host;

            /**
             * @brief Save all dyn_conf parameters. NOTE: When adding/removing
             * params, you must adjust this function, since there is no way to
             * simply dump all parameters (you never know the type and
             * dynamic_reconfigure does not provide documentation)
             *
             * @return Success or failure
             */
            bool writeCurrentConfig()
            {
                const ReconstructionConfig& config = this->config;
                const bfs::path fname = local_box_directory / config_fname;
                ofstream ofs(fname.string(), ios_base::out);

                if (not ofs)
                {
                    return false;
                }
                int width = 30;
                // if you think about shortening this mess, good luck.
                // after your inevitable failure, increment this counter by the
                // hours wasted
                // n_hours_wasted = 5

                // macro for writing each parameter to the yaml stream
                // @param name The parameter name
                // @param quote Whether or not the parameter must be quoted in
                // the output
                #define PARAM(name, quote)\
                    left << setw(width) << #name\
                    << ": " << (quote ? "'" : "")\
                    << config.name << (quote ? "'" : "")
                ofs << boolalpha
                  << PARAM(noExtrusion,          false) << endl
                  << PARAM(intersections,        false) << endl
                  << PARAM(pcm,                  true)  << endl
                  << PARAM(ransac,               false) << endl
                  << PARAM(decomposition,        true)  << endl
                  << PARAM(optimizePlanes,       false) << endl
                  << PARAM(clusterPlanes,        false) << endl
                  << PARAM(cleanContours,        false) << endl
                  << PARAM(planeIterations,      false) << endl
                  << PARAM(fillHoles,            false) << endl
                  << PARAM(rda,                  false) << endl
                  << PARAM(pnt,                  false) << endl
                  << PARAM(smallRegionThreshold, false) << endl
                  << PARAM(kd,                   false) << endl
                  << PARAM(ki,                   false) << endl
                  << PARAM(kn,                   false) << endl
                  << PARAM(mp,                   false) << endl
                  << PARAM(retesselate,          false) << endl
                  << PARAM(lft,                  false) << endl
                  << PARAM(generateTextures,     false) << endl
                  << PARAM(texMinClusterSize,    false) << endl
                  << PARAM(texMaxClusterSize,    false) << endl
                  << PARAM(textureAnalysis,      false) << endl
                  << PARAM(texelSize,            false) << endl
                  << PARAM(classifier,           true)  << endl
                  << PARAM(recalcNormals,        false) << endl
                  << PARAM(threads,              false) << endl
                  << PARAM(sft,                  false) << endl
                  << PARAM(sct,                  false) << endl
                  << PARAM(reductionRatio,       false) << endl
                  << PARAM(tp,                   true)  << endl
                  << PARAM(co,                   true)  << endl
                  << PARAM(nsc,                  false) << endl
                  << PARAM(nccv,                 false) << endl
                  << PARAM(ct,                   false) << endl
                  << PARAM(colt,                 false) << endl
                  << PARAM(stat,                 false) << endl
                  << PARAM(feat,                 false) << endl
                  << PARAM(cro,                  false) << endl
                  << PARAM(patt,                 false) << endl
                  << PARAM(vcfp,                 false) << endl;
                ofs.close();
                return true;
            }

            bool writePLY(const string& tmp_fname, const sensor_msgs::PointCloud2& cloud) const
            {
                // convert to 3dtk coordinate system for SLAM
                const PointCloud<RieglPoint>::Ptr transformed_cloud_ptr  = convert_coords_ros_3dtk(cloud);
                // get temporary file to save cloud to
                PLYWriter writer;
                int res = writer.write(tmp_fname, *transformed_cloud_ptr, true, false);
                return res == 0;
            }

            bool writePose(ros::Time stamp)
            {
                double t[16], ti[16], rP[3], rPT[3];
                bool success = getTransform(t, ti, rP, rPT, this->transform_listener, stamp);
                ofstream ofs((local_box_directory / pose_fname).string());
                if (not ofs)
                {
                    return false;
                }
                ofs << rP[0] << " " << rP[1] << " " << rP[2] << endl <<
                    deg(rPT[0]) << " " << deg(rPT[1]) << " " << deg(rPT[2]);
                ofs.close();
                return true;
            }

            bool writeTriggerFile()
            {
                const bfs::path fname = local_box_directory / trigger_fname;
                ofstream ofs(fname.string(), ios_base::out);
                if (not ofs)
                {
                    return false;
                } else
                {
                    ofs << "Wer das liest, ist doof.\n";
                    ofs.close();
                    return true;
                }
            }

            bool writeStopFile()
            {
                const bfs::path fname = local_box_directory / stop_fname;
                ofstream ofs(fname.string(), ios_base::out);
                if (not ofs)
                {
                    return false;
                } else
                {
                    ofs << "Stop dat shit.\n";
                    ofs.close();
                    return true;
                }
            }


            void reconfigureCallback(lvr_ros::ReconstructionConfig& config, uint32_t level)
            {
                this->config = config;
            }


            void sendCloud(const lvr_ros::SendCloudGoalConstPtr& goal)
            {
                const sensor_msgs::PointCloud2& cloud = goal->cloud;
                bfs::path tmp_fname = local_box_directory /
                    bfs::path(to_string(cloud.header.seq) + string(".ply"));
                ROS_INFO_STREAM("Saving PLY to " << tmp_fname << "...");
                if (not writePLY(tmp_fname.string(), cloud))
                {
                    ROS_ERROR_STREAM("Could not write PLY.");
                    send_as.setAborted();
                    return;
                }
                ROS_INFO_STREAM("Saving current params to temporary file...");
                if (not writeCurrentConfig())
                {
                    ROS_ERROR_STREAM("Could not write current config.");
                    send_as.setAborted();
                    return;
                }
                ROS_INFO_STREAM("Saving pose to temporary file...");
                if (not writePose(cloud.header.stamp))
                {
                    ROS_ERROR_STREAM("Could not write current pose.");
                    send_as.setAborted();
                    return;
                }

                /*******************
                *  Copy ply file  *
                *******************/
                char file_base[20];
                sprintf(file_base, "scan%03d", n_clouds++);
                stringstream command;
                command << "scp ";
                command << tmp_fname;
                command << " " << remote_host << ":" << remote_box_directory / bfs::path(string(file_base)) << ".ply";

                ROS_INFO_STREAM("Executing " << CMD_COLOR(command.str()) << " ...");
                int res = system(command.str().c_str());
                if (res != 0)
                {
                    ROS_ERROR_STREAM("PLY file was not sent successfully. (" << res << ")");
                    send_as.setAborted();
                    return;
                }
                /********************
                *  Copy pose file  *
                ********************/
                command.str(string());
                command.clear();
                command << "scp " << local_box_directory / pose_fname;
                command << " " << remote_host << ":" << remote_box_directory / bfs::path(string(file_base)) << ".pose";

                ROS_INFO_STREAM("Executing " << CMD_COLOR(command.str()) << " ...");
                res = system(command.str().c_str());
                if (res != 0)
                {
                    ROS_ERROR_STREAM("PLY file was not sent successfully. (" << res << ")");
                    send_as.setAborted();
                    return;
                }
                send_as.setSucceeded();

            }

            void stopReconstruction(
                    const lvr_ros::StopReconstructionGoalConstPtr& goal
            )
            {
                ROS_INFO_STREAM("Received stop request.");
                n_clouds = 0;
                if (not writeStopFile())
                {
                    ROS_ERROR_STREAM("Could not create stop file");
                    stop_as.setAborted();
                } else
                {
                    stringstream command;
                    /********************
                     *  Copy stop file  *
                     ********************/
                    command << "scp " << local_box_directory / stop_fname;
                    command << " " << remote_host << ":" << remote_box_directory;

                    ROS_INFO_STREAM("Executing " << CMD_COLOR(command.str()) << " ...");
                    int res = system(command.str().c_str());
                    if (res != 0)
                    {
                        ROS_ERROR_STREAM("stop file was not sent successfully. (" << res << ")");
                        stop_as.setAborted();
                    } else
                    {
                        was_stopped = true;
                        stop_as.setSucceeded();
                        cv.notify_all();
                    }
                }
            }

            void startReconstruction(
                    const lvr_ros::StartReconstructionGoalConstPtr& goal
            )
            {
                n_clouds = 0;
                was_stopped = false;
                if (not writeTriggerFile())
                {
                    ROS_ERROR_STREAM("Could not create trigger file");
                    reconstruct_as.setAborted();
                } else
                {
                    stringstream command;
                    /**********************
                     *  Copy config file  *
                     **********************/
                    command << "scp " << local_box_directory / config_fname;
                    command << " " << remote_host << ":" << remote_box_directory;

                    ROS_INFO_STREAM("Executing " << CMD_COLOR(command.str()) << " ...");

                    int res = system(command.str().c_str());
                    if (res != 0)
                    {
                        ROS_ERROR_STREAM("Could not write config file. (" << res << ")");
                        reconstruct_as.setAborted();
                    } else
                    {
                        command.str(string());
                        command.clear();
                        command << "scp ";
                        command << local_box_directory / trigger_fname;
                        command << " " << remote_host << ":" << remote_box_directory;

                        ROS_INFO_STREAM("Executing " << CMD_COLOR(command.str()) << " ...");

                        res = system(command.str().c_str());
                        if (res != 0)
                        {
                            ROS_ERROR_STREAM("Could not write trigger file. (" << res << ")");
                            send_as.setAborted();
                        } else
                        {
                            ROS_INFO_STREAM("Waiting for Mesh...");
                            unique_lock<mutex> lock(cv_m);
                            // TODO: Wait with timeout, then abort.
                            cv.wait(lock);
                            if (not was_stopped)
                            {
                                // notify called by sendStop()
                                reconstruct_as.setSucceeded();
                                ROS_INFO_STREAM("Reconstruction finished.");
                            } else
                            {
                                reconstruct_as.setAborted();
                                ROS_INFO_STREAM("Reconstruction aborted.");
                            }
                        }
                    }
                }
            }

            void onAnyEvent(const FSEvent& event)
            {

                if (event.file == (local_box_directory / ready_fname))
                {
                    cout << "Waking up..." << endl;
                    cv.notify_all();
                }
            }

            void fileModified(const FSEvent& event)
            {
                this->onAnyEvent(event);
            }

            void fileCreated(const FSEvent& event)
            {
                this->onAnyEvent(event);
            }

            void fileDeleted(const FSEvent& event)
            {
                cout << "File " << event.file << " was deleted." << endl;
            }

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
    };
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "remote_reconstruction");
    lvr_ros::RemoteReconstruction reconstruction;
    ROS_INFO_STREAM("Started action servers.");
    ros::spin();

    return 0;
}
