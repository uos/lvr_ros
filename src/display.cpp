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
 * display.cpp
 *
 * Author: Jan Philipp Vogther <jvogtherr@uni-osnabrueck.de>
 *
 */

#include "lvr_ros/display.h"

namespace lvr_ros
{

/**********************************************************************************************************************/
// Constructor, Destructor

Display::Display()
{
    ros::NodeHandle nh("~");

    // Subscriber
    cloud_subscriber = node_handle.subscribe("/pointcloud", 1, &Display::pointCloudCallback, this);
    mesh_geometry_subscriber = node_handle.subscribe("/mesh_geometry", 1, &Display::meshGeometryCallback, this);

    // Publisher
    mesh_geometry_publisher = node_handle.advertise<mesh_msgs::MeshGeometryStamped>("/display_mesh_geometry", 10);
    mesh_materials_publisher = node_handle.advertise<mesh_msgs::MeshMaterialsStamped>("/display_mesh_materials", 10);
    mesh_vertex_colors_publisher = node_handle.advertise<mesh_msgs::MeshVertexColorsStamped>(
        "/display_mesh_vertex_colors", 10);
    mesh_texture_publisher = node_handle.advertise<sensor_msgs::Image>("/display_mesh_textures", 1000);

    // Service clients
    mesh_geometry_service_client = node_handle.serviceClient<lvr_ros::GetGeometry>("get_geometry");
    mesh_uuid_service_client = node_handle.serviceClient<lvr_ros::GetUUID>("get_uuid");
    mesh_materials_service_client = node_handle.serviceClient<lvr_ros::GetMaterials>("get_materials");
    mesh_vertex_colors_service_client = node_handle.serviceClient<lvr_ros::GetVertexColors>("get_vertex_colors");
    mesh_textures_service_client = node_handle.serviceClient<lvr_ros::GetTexture>("get_texture");

    // Once everything is setup, try to get an initial UUID from lvr_ros::reconstruction
    ROS_INFO("Initial service call");
    lvr_ros::GetUUID srv_uuid;
    if (mesh_uuid_service_client.call(srv_uuid))
    {
        std::string uuid = (std::string)srv_uuid.response.uuid;
        ROS_INFO_STREAM("Got initial mesh with UUID=" << uuid);
        processNewUUID(uuid);
    }
    else
    {
        ROS_INFO("No initial data available. Waiting for callback to trigger ...");
    }
}

Display::~Display() {}

/**********************************************************************************************************************/
// Callbacks

void Display::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr cloud)
{
    // ROS_INFO("PointCloud2 received");
    // TODO: colorize cloud, dann reconstruct action
}

void Display::meshGeometryCallback(const mesh_msgs::MeshGeometryStamped::ConstPtr mesh_geometry_stamped)
{
    ROS_INFO("MeshGeometry received");
    processNewGeometry(*mesh_geometry_stamped);
}

/**********************************************************************************************************************/
// Publisher logic

void Display::processNewUUID(std::string uuid)
{
    lvr_ros::GetGeometry srv_geometry;
    srv_geometry.request.uuid = uuid;
    if (mesh_geometry_service_client.call(srv_geometry))
    {
        processNewGeometry((mesh_msgs::MeshGeometryStamped)srv_geometry.response.mesh_geometry_stamped);
    }
    else
    {
        ROS_INFO("Error while calling geometry service");
    }
}

void Display::processNewGeometry(const mesh_msgs::MeshGeometryStamped mesh_geometry_stamped)
{

    std::string uuid = mesh_geometry_stamped.uuid;
    ROS_INFO_STREAM("Processing mesh with UUID=" << uuid);

    // New geometry received, now call lvr_ros services to retrieve materials, vertex colors and textures
    lvr_ros::GetMaterials srv_materials;
    lvr_ros::GetVertexColors srv_vertexColors;
    lvr_ros::GetTexture srv_texture;

    srv_materials.request.uuid = uuid;
    srv_vertexColors.request.uuid = uuid;
    srv_texture.request.uuid = uuid;

    mesh_msgs::MeshMaterialsStamped mesh_materials_stamped;
    mesh_msgs::MeshVertexColorsStamped mesh_vertex_colors_stamped;

    // Call materials service
    if (mesh_materials_service_client.call(srv_materials))
    {
        mesh_materials_stamped = (mesh_msgs::MeshMaterialsStamped)srv_materials.response.mesh_materials_stamped;
        // Publish materials
        mesh_materials_publisher.publish(mesh_materials_stamped);
    }
    else
    {
        ROS_ERROR("Error while calling GetMaterials service");
        return;
    }

    // Call vertex colors service
    if (mesh_vertex_colors_service_client.call(srv_vertexColors))
    {
        mesh_vertex_colors_stamped =
            (mesh_msgs::MeshVertexColorsStamped)srv_vertexColors.response.mesh_vertex_colors_stamped;
        // Publish vertex colors
        mesh_vertex_colors_publisher.publish(mesh_vertex_colors_stamped);
    }
    else
    {
        ROS_ERROR("Error while calling GetVertexColors service");
        return;
    }

    // Call texture service (multiple time: once for each texture index found in materials)
    for (unsigned int i = 0; i < mesh_materials_stamped.mesh_materials.materials.size(); i++)
    {
        if (mesh_materials_stamped.mesh_materials.materials[i].has_texture)
        {
            srv_texture.request.texture_index = mesh_materials_stamped.mesh_materials.materials[i].texture_index;
            if (mesh_textures_service_client.call(srv_texture))
            {
                // Publish texture
                mesh_texture_publisher.publish((sensor_msgs::Image)srv_texture.response.texture);
            }
            else
            {
                ROS_ERROR("Error while calling GetTexture service");
                return;
            }

        }
    }

    ROS_INFO("Successfully distributed mesh messages");
}

/**********************************************************************************************************************/
// End of namespace & Main

} /* namespace lvr_ros */

int main(int argc, char **args)
{
    ros::init(argc, args, "display");
    lvr_ros::Display display;
    ros::spin();

    return 0;
}
