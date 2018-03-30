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
 * hdf5_to_msg.h
 *
 * Author: Jan Philipp Vogtherr <jvogtherr@uos.de>
 *
 */

#ifndef hdf5_to_msg_H_
#define hdf5_to_msg_H_


// #justROSthings
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

// hdf5
#include <lvr2/io/PlutoMapIO.hpp>

// services & data types
#include <mesh_msgs/ClusterLabel.h>
#include <mesh_msgs/GetGeometry.h>
#include <mesh_msgs/GetMaterials.h>
#include <mesh_msgs/GetTexture.h>
#include <mesh_msgs/GetUUID.h>
#include <mesh_msgs/GetVertexColors.h>
#include <mesh_msgs/GetVertexCosts.h>
#include <mesh_msgs/GetLabeledClusters.h>
#include <label_manager/GetLabelGroups.h>
#include <label_manager/GetLabeledClusterGroup.h>
#include <label_manager/DeleteLabel.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/fill_image.h>

// boost
#include <boost/algorithm/string.hpp>

namespace lvr_ros
{

class hdf5_to_msg
{

public:
	hdf5_to_msg();
	~hdf5_to_msg() {};

protected:
	// Mesh services
    bool service_getGeometry(
    	mesh_msgs::GetGeometry::Request& req,
    	mesh_msgs::GetGeometry::Response& res);
    bool service_getGeometryVertices(
    	mesh_msgs::GetGeometry::Request& req,
    	mesh_msgs::GetGeometry::Response& res);
    bool service_getGeometryFaces(
    	mesh_msgs::GetGeometry::Request& req,
    	mesh_msgs::GetGeometry::Response& res);
    bool service_getGeometryVertexnormals(
    	mesh_msgs::GetGeometry::Request& req,
    	mesh_msgs::GetGeometry::Response& res);

    bool service_getMaterials(
    	mesh_msgs::GetMaterials::Request& req,
    	mesh_msgs::GetMaterials::Response& res);
    bool service_getTexture(
    	mesh_msgs::GetTexture::Request& req,
    	mesh_msgs::GetTexture::Response& res);
    bool service_getUUID(
    	mesh_msgs::GetUUID::Request& req,
    	mesh_msgs::GetUUID::Response& res);
    bool service_getVertexColors(
    	mesh_msgs::GetVertexColors::Request& req,
    	mesh_msgs::GetVertexColors::Response& res);

    // Label manager services
    bool service_getLabeledClusters(
        mesh_msgs::GetLabeledClusters::Request& req,
        mesh_msgs::GetLabeledClusters::Response& res);
    bool service_getLabelGroups(
        label_manager::GetLabelGroups::Request& req,
        label_manager::GetLabelGroups::Response& res);
    bool service_getLabeledClusterGroup(
        label_manager::GetLabeledClusterGroup::Request& req,
        label_manager::GetLabeledClusterGroup::Response& res);
    bool service_deleteLabel(
        label_manager::DeleteLabel::Request& req,
        label_manager::DeleteLabel::Response& res);

    // Vertex costs
    bool service_getRoughness(
        mesh_msgs::GetVertexCosts::Request& req,
        mesh_msgs::GetVertexCosts::Response& res);
    bool service_getHeightDifference(
        mesh_msgs::GetVertexCosts::Request& req,
        mesh_msgs::GetVertexCosts::Response& res);


    void callback_clusterLabel(const mesh_msgs::ClusterLabel::ConstPtr& msg);


private:

    // Mesh message service servers
    ros::ServiceServer srv_get_geometry_;
    ros::ServiceServer srv_get_geometry_vertices_;
    ros::ServiceServer srv_get_geometry_faces_;
    ros::ServiceServer srv_get_geometry_vertex_normals_;
    ros::ServiceServer srv_get_materials_;
    ros::ServiceServer srv_get_texture_;
    ros::ServiceServer srv_get_uuid_;
    ros::ServiceServer srv_get_vertex_colors_;
    ros::ServiceServer srv_get_roughness_;
    ros::ServiceServer srv_get_height_difference_;

    // Label manager services and subs/pubs
    ros::Subscriber sub_cluster_label_;
    ros::Publisher pub_cluster_label_;
    ros::ServiceServer srv_get_labeled_clusters_;
    ros::ServiceServer srv_get_label_groups_;
    ros::ServiceServer srv_get_labeled_cluster_group_;
    ros::ServiceServer srv_delete_label_;

    // ROS
    ros::NodeHandle node_handle;

    // ROS parameter
    std::string inputFile;

    std::string mesh_uuid = "mesh";

};



} // end namespace

#endif
