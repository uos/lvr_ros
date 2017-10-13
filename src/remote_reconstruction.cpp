#include "lvr_ros/ReconstructionConfig.h"
#include "lvr_ros/SendCloudAction.h"
#include "lvr_ros/StartReconstructionAction.h"

// no hyperspectral_calibration prefix ??
#include "types/types.hpp"

#include <fstream>
#include <cstdlib>
#include <actionlib/server/simple_action_server.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <mesh_msgs/TriangleMesh.h>
#include <mesh_msgs/TriangleMeshStamped.h>

#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <boost/filesystem/path.hpp>

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

    static const bfs::path remote_box_direcotry("/tmp/clouds_remote");
    static const bfs::path local_box_direcotry("/tmp/clouds_local");
    static const string trigger_fname = ".start_reconstruction";


    class RemoteReconstruction
    {
        public:
            RemoteReconstruction() : send_as(node_handle, "send",
                    boost::bind(&RemoteReconstruction::sendCloud, this, _1), false),
            reconstruct_as(node_handle, "reconstruct",
                    boost::bind(&RemoteReconstruction::startReconstruction, this, _1), false)
        {
            cloud_subscriber = node_handle.subscribe(
                    "/pointcloud",
                    1,
                    &RemoteReconstruction::pointCloudCallback,
                    this
            );

            mesh_publisher = node_handle.advertise<mesh_msgs::TriangleMeshStamped>("/mesh", 1);

            // setup dynamic reconfigure
            reconfigure_server_ptr = DynReconfigureServerPtr(new DynReconfigureServer(node_handle));
            callback_type = boost::bind(&RemoteReconstruction::reconfigureCallback, this, _1, _2);
            reconfigure_server_ptr->setCallback(callback_type);

            send_as.start();
            reconstruct_as.start();

            if (not bfs::exists(remote_box_direcotry))
            {
                bfs::create_directory(remote_box_direcotry);
            }
        }

        private:

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
                ofstream ofs("remote_reconstruction_config.yaml", ios_base::out);

                if (not ofs)
                {
                    return false;
                }
                int width = 30;
                ofs << left << setw(width) << "voxelsize:"                 << config.voxelsize                 << endl;
                ofs << left << setw(width) << "noExtrusion:"               << config.noExtrusion               << endl;
                ofs << left << setw(width) << "intersections:"             << config.intersections             << endl;
                ofs << left << setw(width) << "pcm:"                       << "'" << config.pcm << "'"          << endl;
                ofs << left << setw(width) << "ransac:"                    << config.ransac                    << endl;
                ofs << left << setw(width) << "decomposition:"             << "'" << config.decomposition << "'" << endl;
                ofs << left << setw(width) << "optimizePlanes:"            << config.optimizePlanes            << endl;
                ofs << left << setw(width) << "clusterPlanes:"             << config.clusterPlanes             << endl;
                ofs << left << setw(width) << "cleanContours:"             << config.cleanContours             << endl;
                ofs << left << setw(width) << "planeIterations:"           << config.planeIterations           << endl;
                ofs << left << setw(width) << "fillHoles:"                 << config.fillHoles                 << endl;
                ofs << left << setw(width) << "danglingArtifacts:"         << config.danglingArtifacts         << endl;
                ofs << left << setw(width) << "normalThreshold:"           << config.normalThreshold           << endl;
                ofs << left << setw(width) << "smallRegionThreshold:"      << config.smallRegionThreshold      << endl;
                ofs << left << setw(width) << "kd:"                        << config.kd                        << endl;
                ofs << left << setw(width) << "ki:"                        << config.ki                        << endl;
                ofs << left << setw(width) << "kn:"                        << config.kn                        << endl;
                ofs << left << setw(width) << "minPlaneSize:"              << config.minPlaneSize              << endl;
                ofs << left << setw(width) << "retesselate:"               << config.retesselate               << endl;
                ofs << left << setw(width) << "lineFusionThreshold:"       << config.lineFusionThreshold       << endl;
                ofs << left << setw(width) << "generateTextures:"          << config.generateTextures          << endl;
                ofs << left << setw(width) << "textureAnalysis:"           << config.textureAnalysis           << endl;
                ofs << left << setw(width) << "writeClassificationResult:" << config.writeClassificationResult << endl;
                ofs << left << setw(width) << "texelSize:"                 << config.texelSize                 << endl;
                ofs << left << setw(width) << "classifier:"                << "'" << config.classifier  << "'" << endl;
                ofs << left << setw(width) << "depth:"                     << config.depth                     << endl;
                ofs << left << setw(width) << "recalcNormals:"             << config.recalcNormals             << endl;
                ofs << left << setw(width) << "threads:"                   << config.threads                   << endl;
                ofs << left << setw(width) << "sharpFeatThreshold:"        << config.sharpFeatThreshold        << endl;
                ofs << left << setw(width) << "sharpCornThreshold:"        << config.sharpCornThreshold        << endl;
                ofs << left << setw(width) << "ecm:"                       << config.ecm                       << endl;
                ofs << left << setw(width) << "numEdgeCollapses:"          << config.numEdgeCollapses          << endl;
                ofs << left << setw(width) << "texturePack:"               << "'" << config.texturePack << "'" << endl;
                ofs << left << setw(width) << "numStatsColors:"            << config.numStatsColors            << endl;
                ofs << left << setw(width) << "numCCVColors:"              << config.numCCVColors              << endl;
                ofs << left << setw(width) << "coherenceThreshold:"        << config.coherenceThreshold        << endl;
                ofs << left << setw(width) << "useCrossCorr:"              << config.useCrossCorr              << endl;
                ofs << left << setw(width) << "patternThreshold:"          << config.patternThreshold          << endl;
                ofs << left << setw(width) << "minTransformVotes:"         << config.minTransformVotes         << endl;
                ofs.close();
                return true;
            }

            bool writePLY(const string& tmp_fname, const sensor_msgs::PointCloud2& cloud) const
            {
                PointCloud<RieglPoint> pcl_cloud;
                fromROSMsg<RieglPoint>(cloud, pcl_cloud);
                // get temporary file to save cloud to
                ROS_INFO_STREAM("Saving PLY to temporary file...");
                PLYWriter writer;
                //                             write in binary
                //                                   |
                int res = writer.write(tmp_fname, pcl_cloud, true, false);
                return res == 0;
            }

            bool writePose(const string fname, const geometry_msgs::PoseStamped& pose)
            {
                ofstream ofs(fname, ios_base::out);
                if (not ofs)
                {
                    return false;
                } else
                {
                    return false;
                }
            }

            bool writeTriggerFile()
            {
                ofstream ofs(trigger_fname, ios_base::out);
                if (not ofs)
                {
                    return false;
                } else
                {
                    ofs << "settings: wueva\n";
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
                std::string tmp_fname = std::tmpnam(nullptr);
                if (not writePLY(tmp_fname, cloud))
                {
                    ROS_ERROR_STREAM("Coudl not write PLY.");
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

                string command("scp ");
                command += string(tmp_fname);
                command += string(" localhost:") + remote_box_direcotry.string();

                ROS_INFO_STREAM("Executing '" << command << "'...");
                int res = system(command.c_str());
                if (res != 0)
                {
                    ROS_ERROR_STREAM("PLY file was not sent successfully. (" << res << ")");
                    send_as.setAborted();
                } else
                {
                    send_as.setSucceeded();
                }


            }


            void waitForResult()
            {

            }


            void startReconstruction(
                    const lvr_ros::StartReconstructionGoalConstPtr& goal
            )
            {
                if (not writeTriggerFile())
                {
                    ROS_ERROR_STREAM("Could not create trigger file");
                    reconstruct_as.setAborted();
                } else
                {
                    string command("scp ");
                    command += string(trigger_fname);
                    command += string(" localhost:") + remote_box_direcotry.string();

                    ROS_INFO_STREAM("Executing '" << command << "'...");
                    int res = system(command.c_str());
                    if (res != 0)
                    {
                        ROS_ERROR_STREAM("Could not transfer trigger file. (" << res << ")");
                        reconstruct_as.setAborted();
                    } else
                    {
                        reconstruct_as.setSucceeded();
                    }
                }

                waitForResult();
            }

            void pointCloudCallback(
                    const sensor_msgs::PointCloud2::ConstPtr& cloud
            )
            {
            }

            DynReconfigureServerPtr reconfigure_server_ptr;
            DynReconfigureServer::CallbackType callback_type;

            ros::NodeHandle node_handle;
            ros::Publisher mesh_publisher;
            ros::Subscriber cloud_subscriber;
            ReconstructionConfig config;
            SendCloudActionServer send_as;
            StartReconstructionActionServer reconstruct_as;

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
