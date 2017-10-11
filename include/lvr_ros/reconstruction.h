/*
 * UOS-ROS packages - Robot Operating System code by the University of Osnabrück
 * Copyright (C) 2013 University of Osnabrück
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 * reconstruction.h
 *
 * Author: Sebastian Pütz <spuetz@uos.de>,
 *
 */

#ifndef LVR_ROS_RECONSTRUCTION_H_
#define LVR_ROS_RECONSTRUCTION_H_

#include <actionlib/server/simple_action_server.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <dynamic_reconfigure/server.h>
#include "lvr_ros/ReconstructionConfig.h"
#include "lvr_ros/ReconstructAction.h"


#include <mesh_msgs/TriangleMesh.h>
#include <mesh_msgs/TriangleMeshStamped.h>

#include <lvr2/geometry/BaseVector.hpp>
#include <lvr2/io/PointBuffer.hpp>
#include <lvr2/io/MeshBuffer.hpp>
#include <lvr/io/PointBuffer.hpp>
#include <lvr/io/MeshBuffer.hpp>


namespace lvr_ros
{

using Vec = lvr2::BaseVector<float>;
using PointBuffer = lvr2::PointBuffer<Vec>;
using PointBufferPtr = lvr2::PointBufferPtr<Vec>;
// using MeshBuffer = lvr2::MeshBuffer<Vec>;
// using MeshBufferPtr = lvr2::MeshBufferPtr<Vec>;


class Reconstruction
{
public:
    Reconstruction();

private:
    void reconstruct(const lvr_ros::ReconstructGoalConstPtr& goal);
    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud);
    bool createMesh(const sensor_msgs::PointCloud2& cloud, mesh_msgs::TriangleMeshStamped& mesh);
    bool createMesh(PointBufferPtr& point_buffer, lvr::MeshBufferPtr& mesh_buffer);
    float *getStatsCoeffs(std::string filename) const;
    void reconfigureCallback(lvr_ros::ReconstructionConfig& config, uint32_t level);

    typedef dynamic_reconfigure::Server <lvr_ros::ReconstructionConfig> DynReconfigureServer;
    typedef boost::shared_ptr <DynReconfigureServer> DynReconfigureServerPtr;
    typedef actionlib::SimpleActionServer<lvr_ros::ReconstructAction> ActionServer;

    DynReconfigureServerPtr reconfigure_server_ptr;
    DynReconfigureServer::CallbackType callback_type;

    ros::NodeHandle node_handle;
    ros::Publisher mesh_publisher;
    ros::Subscriber cloud_subscriber;
    ReconstructionConfig config;
    ActionServer as_;

};

} // namespace lvr_ros

#endif /* LVR_ROS_RECONSTRUCTION_H_ */
