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
    mesh_texture_publisher = node_handle.advertise<mesh_msgs::Texture>("/display_mesh_textures", 1000);

    // Service clients
    mesh_geometry_service_client = node_handle.serviceClient<lvr_ros::GetGeometry>("get_geometry");
    mesh_uuid_service_client = node_handle.serviceClient<lvr_ros::GetUUID>("get_uuid");
    mesh_materials_service_client = node_handle.serviceClient<lvr_ros::GetMaterials>("get_materials");
    mesh_vertex_colors_service_client = node_handle.serviceClient<lvr_ros::GetVertexColors>("get_vertex_colors");
    mesh_textures_service_client = node_handle.serviceClient<lvr_ros::GetTexture>("get_texture");

    int numSubscribers = 0;
    while (numSubscribers < 1)
    {
        ROS_INFO("Waiting for subscribers");
        ros::Duration(1.0).sleep();

        numSubscribers = mesh_geometry_publisher.getNumSubscribers()
            + mesh_materials_publisher.getNumSubscribers()
            + mesh_vertex_colors_publisher.getNumSubscribers()
            + mesh_texture_publisher.getNumSubscribers();

        if (numSubscribers > 0)
        {
            ROS_INFO_STREAM("Found " << numSubscribers << " subscribers");
        }
    }

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

void Display::publish()
{
    if (publish_flag)
    {
        if (has_geom)
        {
            mesh_geometry_publisher.publish(cache_geometry);
            ROS_INFO_STREAM("Published geometry for UUID=" << cache_geometry.uuid);
        }
        if (has_mats)
        {
            mesh_materials_publisher.publish(cache_materials);
            ROS_INFO_STREAM("Published materials for UUID=" << cache_materials.uuid);
        }
        if (has_vcs)
        {
            mesh_vertex_colors_publisher.publish(cache_vertexcolors);
            ROS_INFO_STREAM("Published vertex colors for UUID=" << cache_vertexcolors.uuid);
        }
        if (has_tex)
        {
            for (unsigned int i = 0; i < cache_textures.size(); i++)
            {
                mesh_texture_publisher.publish(cache_textures.at(i));
                ROS_INFO_STREAM("Published texture with ID=" << cache_textures.at(i).texture_index << " for UUID="
                    << cache_textures.at(i).uuid);
            }
        }
        publish_flag = false;
    }
}

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
    // Reset cache
    has_geom = false;
    has_mats = false;
    has_vcs = false;
    has_tex = false;

    std::string uuid = mesh_geometry_stamped.uuid;
    ROS_INFO_STREAM("Processing mesh with UUID=" << uuid);

    // Cache geometry
    cache_geometry = mesh_geometry_stamped;
    has_geom = true;

    // New geometry received, now call lvr_ros services to retrieve materials, vertex colors and textures
    lvr_ros::GetMaterials srv_materials;
    lvr_ros::GetVertexColors srv_vertexColors;
    lvr_ros::GetTexture srv_texture;

    // Prepare service calls
    srv_materials.request.uuid = uuid;
    srv_vertexColors.request.uuid = uuid;
    srv_texture.request.uuid = uuid;

    // Call materials service
    if (mesh_materials_service_client.call(srv_materials))
    {
        cache_materials = (mesh_msgs::MeshMaterialsStamped)srv_materials.response.mesh_materials_stamped;
        has_mats = true;
    }
    else
    {
        ROS_ERROR("Error while calling GetMaterials service");
        return;
    }

    // Call vertex colors service
    if (mesh_vertex_colors_service_client.call(srv_vertexColors))
    {
        cache_vertexcolors = (mesh_msgs::MeshVertexColorsStamped)srv_vertexColors.response.mesh_vertex_colors_stamped;
        has_vcs = true;
    }
    else
    {
        ROS_ERROR("Error while calling GetVertexColors service");
        return;
    }

    if (has_mats)
    {
        cache_textures.clear();
        // Call texture service (multiple time: once for each texture index found in materials)
        for (unsigned int i = 0; i < cache_materials.mesh_materials.materials.size(); i++)
        {
            if (cache_materials.mesh_materials.materials[i].has_texture)
            {
                srv_texture.request.texture_index = cache_materials.mesh_materials.materials[i].texture_index;
                if (mesh_textures_service_client.call(srv_texture))
                {
                    cache_textures.push_back((mesh_msgs::Texture)srv_texture.response.texture);
                }
                else
                {
                    ROS_ERROR("Error while calling GetTexture service");
                    return;
                }

            }
        }
        if (cache_textures.size() > 0)
        {
            has_tex = true;
        }
    }

    publish_flag = true;
    ROS_INFO("Successfully cached mesh messages");
}

/**********************************************************************************************************************/
// End of namespace & Main

} /* namespace lvr_ros */

int main(int argc, char **args)
{
    ros::init(argc, args, "display");
    lvr_ros::Display display;
    ros::Rate rate(500);
    while (ros::ok())
    {
        rate.sleep();
        display.publish();
        ros::spinOnce();
    }

    return 0;
}
