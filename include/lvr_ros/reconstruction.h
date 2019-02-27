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
#include <mesh_msgs/GetGeometry.h>
#include <mesh_msgs/GetMaterials.h>
#include <mesh_msgs/GetTexture.h>
#include <mesh_msgs/GetUUID.h>
#include <mesh_msgs/GetVertexColors.h>
#include <mesh_msgs/GetVertexCosts.h>

#include <mesh_msgs/TriangleMesh.h>
#include <mesh_msgs/TriangleMeshStamped.h>
#include <mesh_msgs/MeshTexture.h>

#include <lvr2/geometry/BaseVector.hpp>
#include <lvr2/io/PointBuffer.hpp>
#include <lvr2/io/MeshBuffer.hpp>
#include <lvr2/io/PointBuffer.hpp>
#include <lvr2/io/MeshBuffer.hpp>


namespace lvr_ros
{

using Vec = lvr2::BaseVector<float>;
using PointBuffer = lvr2::PointBuffer;
using PointBufferPtr = lvr2::PointBufferPtr;
// using MeshBuffer = lvr2::MeshBuffer<Vec>;
// using MeshBufferPtr = lvr2::MeshBufferPtr<Vec>;


class Reconstruction
{
public:
    Reconstruction();

private:

    /**
     * Reconstruct action callback
     *
     * The result of this actionis a TriangleMesh (old, deprecated) and a UUID string that identifies an instance
     * of a mesh reconstruction.
     *
     * The caller of this action should use the UUID to call the services that this node offers to receive the
     * corresponding mesh geometry and attributes. For migration from one message format to another, this action will offer
     * both versions.
     * Make sure to migrate to the new message format quickly before the old one gets discontinued eventually.
     */
    void reconstruct(const lvr_ros::ReconstructGoalConstPtr& goal);

    // Service callbacks
    bool service_getGeometry(mesh_msgs::GetGeometry::Request& req, mesh_msgs::GetGeometry::Response& res);
    bool service_getMaterials(mesh_msgs::GetMaterials::Request& req, mesh_msgs::GetMaterials::Response& res);
    bool service_getTexture(mesh_msgs::GetTexture::Request& req, mesh_msgs::GetTexture::Response& res);
    bool service_getUUID(mesh_msgs::GetUUID::Request& req, mesh_msgs::GetUUID::Response& res);
    bool service_getVertexColors(mesh_msgs::GetVertexColors::Request& req, mesh_msgs::GetVertexColors::Response& res);

    // Subscriber callback
    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud);

    /**
     * This method will generate
     *   - a TriangleMesh message
     *       - this message will be published in the callback or action function that is calling this method
     *   - a MeshGeometry message and all corresponding MeshAttribute messages
     *       - these messages will be cached and will be available via a service
     *
     * Please note: For future versions, it is not intended to keep both messages around. TriangleMesh will be
     * discontinued in favor of the new message structure. To ensure a smooth transition between both APIs, this
     * version of LVR_ROS will be able to generate both messages.
     */
    bool createMeshMessageFromPointCloud(const sensor_msgs::PointCloud2& cloud, mesh_msgs::TriangleMeshStamped& mesh);
    bool createMeshBufferFromPointBuffer(PointBufferPtr& point_buffer, lvr2::MeshBufferPtr& mesh_buffer);

    // Utility
    float *getStatsCoeffs(std::string filename) const;
    void reconfigureCallback(lvr_ros::ReconstructionConfig& config, uint32_t level);
    typedef dynamic_reconfigure::Server <lvr_ros::ReconstructionConfig> DynReconfigureServer;
    typedef boost::shared_ptr <DynReconfigureServer> DynReconfigureServerPtr;
    typedef actionlib::SimpleActionServer<lvr_ros::ReconstructAction> ActionServer;
    DynReconfigureServerPtr reconfigure_server_ptr;
    DynReconfigureServer::CallbackType callback_type;

    // Node, Publishers, Subscribers, Config
    ros::NodeHandle node_handle;
    ros::Publisher mesh_publisher;          // Is used to publish old TriangleMesh
    ros::Publisher mesh_geometry_publisher; // Is used to publish new MeshGeometry
    ros::Subscriber cloud_subscriber;
    ReconstructionConfig config;

    // ActionServer and Services
    ActionServer as_;
    ros::ServiceServer srv_get_geometry_;
    ros::ServiceServer srv_get_materials_;
    ros::ServiceServer srv_get_texture_;
    ros::ServiceServer srv_get_uuid_;
    ros::ServiceServer srv_get_vertex_colors_;

    // ROS message cache
    // Reconstruction will write these messages to cache, services will send them
    bool cache_initialized = false;
    mesh_msgs::MeshGeometryStamped cache_mesh_geometry_stamped;
    mesh_msgs::MeshMaterialsStamped cache_mesh_materials_stamped;
    mesh_msgs::MeshVertexColorsStamped cache_mesh_vertex_colors_stamped;
    std::string cache_uuid;
    std::vector<mesh_msgs::MeshTexture> cache_textures;

};

} // namespace lvr_ros

#endif /* LVR_ROS_RECONSTRUCTION_H_ */
