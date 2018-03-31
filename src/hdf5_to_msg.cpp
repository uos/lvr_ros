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
 * Author: Jan Philipp Vogtherr <jvogtherr@uni-osnabrueck.de>
 *
 */

#include "lvr_ros/hdf5_to_msg.h"

namespace lvr_ros {

// ===================================================================================================================


hdf5_to_msg::hdf5_to_msg()
{
    ros::NodeHandle nh("~");

    if (!nh.getParam("inputFile", inputFile))
    {
        inputFile = "/tmp/pluto/map.h5";
    }

    ROS_INFO_STREAM("Using input file: " << inputFile);

    srv_get_geometry_ = node_handle.advertiseService(
        "get_geometry", &hdf5_to_msg::service_getGeometry, this);
    srv_get_geometry_vertices_ = node_handle.advertiseService(
        "get_geometry_vertices", &hdf5_to_msg::service_getGeometryVertices, this);
    srv_get_geometry_faces_ = node_handle.advertiseService(
        "get_geometry_faces", &hdf5_to_msg::service_getGeometryFaces, this);
     srv_get_geometry_vertex_normals_ = node_handle.advertiseService(
        "get_geometry_vertexnormals", &hdf5_to_msg::service_getGeometryVertexnormals, this);
    srv_get_materials_ = node_handle.advertiseService(
        "get_materials", &hdf5_to_msg::service_getMaterials, this);
    srv_get_texture_ = node_handle.advertiseService(
        "get_texture", &hdf5_to_msg::service_getTexture, this);
    srv_get_uuid_ = node_handle.advertiseService(
        "get_uuid", &hdf5_to_msg::service_getUUID, this);
    srv_get_vertex_colors_ = node_handle.advertiseService(
        "get_vertex_colors", &hdf5_to_msg::service_getVertexColors, this);

    srv_get_labeled_clusters_ = node_handle.advertiseService(
        "get_labeled_clusters", &hdf5_to_msg::service_getLabeledClusters, this);
    srv_get_label_groups_ = node_handle.advertiseService(
        "get_label_groups", &hdf5_to_msg::service_getLabelGroups, this);
    srv_get_labeled_cluster_group_ = node_handle.advertiseService(
        "get_labeled_cluster_group", &hdf5_to_msg::service_getLabeledClusterGroup, this);
    srv_delete_label_ = node_handle.advertiseService(
        "delete_label", &hdf5_to_msg::service_deleteLabel, this);

    srv_get_roughness_ = node_handle.advertiseService(
        "get_roughness", &hdf5_to_msg::service_getRoughness, this);
    srv_get_height_difference_ = node_handle.advertiseService(
        "get_height_difference", &hdf5_to_msg::service_getHeightDifference, this);

    sub_cluster_label_ = node_handle.subscribe("cluster_label", 10, &hdf5_to_msg::callback_clusterLabel, this);
    pub_cluster_label_ = node_handle.advertise<mesh_msgs::Cluster>("new_cluster_label", 1);

}

bool hdf5_to_msg::service_getUUID(
    mesh_msgs::GetUUID::Request& req,
    mesh_msgs::GetUUID::Response& res)
{
    ROS_INFO("Get UUID");
    res.uuid = mesh_uuid;
    return true;
}

bool hdf5_to_msg::service_getGeometry(
    mesh_msgs::GetGeometry::Request& req,
    mesh_msgs::GetGeometry::Response& res)
{
    ROS_INFO("Get geometry");
    lvr2::PlutoMapIO pmio(inputFile);


    // Vertices
    auto vertices = pmio.getVertices();
    unsigned int nVertices = vertices.size() / 3;
    ROS_INFO_STREAM("Found " << nVertices << " vertices");
    res.mesh_geometry_stamped.mesh_geometry.vertices.resize(nVertices);
    for (unsigned int i = 0; i < nVertices; i++)
    {
        res.mesh_geometry_stamped.mesh_geometry.vertices[i].x = vertices[i * 3];
        res.mesh_geometry_stamped.mesh_geometry.vertices[i].y = vertices[i * 3 + 1];
        res.mesh_geometry_stamped.mesh_geometry.vertices[i].z = vertices[i * 3 + 2];
    }

    // Vertex normals
    auto vertexNormals = pmio.getVertexNormals();
    unsigned int nVertexNormals = vertexNormals.size() / 3;
    ROS_INFO_STREAM("Found " << nVertexNormals << " vertex normals");
    res.mesh_geometry_stamped.mesh_geometry.vertex_normals.resize(nVertexNormals);
    for (unsigned int i = 0; i < nVertexNormals; i++)
    {
        res.mesh_geometry_stamped.mesh_geometry.vertex_normals[i].x = vertexNormals[i * 3];
        res.mesh_geometry_stamped.mesh_geometry.vertex_normals[i].y = vertexNormals[i * 3 + 1];
        res.mesh_geometry_stamped.mesh_geometry.vertex_normals[i].z = vertexNormals[i * 3 + 2];
    }

    // Faces
    auto faceIds = pmio.getFaceIds();
    unsigned int nFaces = faceIds.size() / 3;
    ROS_INFO_STREAM("Found " << nFaces << " faces");
    res.mesh_geometry_stamped.mesh_geometry.faces.resize(nFaces);
    for (unsigned int i = 0; i < nFaces; i++)
    {
        res.mesh_geometry_stamped.mesh_geometry.faces[i].vertex_indices[0] = faceIds[i * 3];
        res.mesh_geometry_stamped.mesh_geometry.faces[i].vertex_indices[1] = faceIds[i * 3 + 1];
        res.mesh_geometry_stamped.mesh_geometry.faces[i].vertex_indices[2] = faceIds[i * 3 + 2];
    }

    // Header
    res.mesh_geometry_stamped.uuid = mesh_uuid;
    res.mesh_geometry_stamped.header.frame_id = "map";
    res.mesh_geometry_stamped.header.stamp = ros::Time::now();

    ROS_INFO("Done");

    return true;
}

bool hdf5_to_msg::service_getGeometryVertices(
    mesh_msgs::GetGeometry::Request& req,
    mesh_msgs::GetGeometry::Response& res)
{
    ROS_INFO("Get geometry vertices");
    lvr2::PlutoMapIO pmio(inputFile);

    // Vertices
    auto vertices = pmio.getVertices();
    unsigned int nVertices = vertices.size() / 3;
    ROS_INFO_STREAM("Found " << nVertices << " vertices");
    res.mesh_geometry_stamped.mesh_geometry.vertices.resize(nVertices);
    for (unsigned int i = 0; i < nVertices; i++)
    {
        res.mesh_geometry_stamped.mesh_geometry.vertices[i].x = vertices[i * 3];
        res.mesh_geometry_stamped.mesh_geometry.vertices[i].y = vertices[i * 3 + 1];
        res.mesh_geometry_stamped.mesh_geometry.vertices[i].z = vertices[i * 3 + 2];
    }

    // Header
    res.mesh_geometry_stamped.uuid = mesh_uuid;
    res.mesh_geometry_stamped.header.frame_id = "map";
    res.mesh_geometry_stamped.header.stamp = ros::Time::now();

    ROS_INFO("Done");

    return true;
}

bool hdf5_to_msg::service_getGeometryFaces(
    mesh_msgs::GetGeometry::Request& req,
    mesh_msgs::GetGeometry::Response& res)
{
    ROS_INFO("Get geometry faces");
    lvr2::PlutoMapIO pmio(inputFile);

    // Faces
    auto faceIds = pmio.getFaceIds();
    unsigned int nFaces = faceIds.size() / 3;
    ROS_INFO_STREAM("Found " << nFaces << " faces");
    res.mesh_geometry_stamped.mesh_geometry.faces.resize(nFaces);
    for (unsigned int i = 0; i < nFaces; i++)
    {
        res.mesh_geometry_stamped.mesh_geometry.faces[i].vertex_indices[0] = faceIds[i * 3];
        res.mesh_geometry_stamped.mesh_geometry.faces[i].vertex_indices[1] = faceIds[i * 3 + 1];
        res.mesh_geometry_stamped.mesh_geometry.faces[i].vertex_indices[2] = faceIds[i * 3 + 2];
    }

    // Header
    res.mesh_geometry_stamped.uuid = mesh_uuid;
    res.mesh_geometry_stamped.header.frame_id = "map";
    res.mesh_geometry_stamped.header.stamp = ros::Time::now();

    ROS_INFO("Done");

    return true;
}

bool hdf5_to_msg::service_getGeometryVertexnormals(
    mesh_msgs::GetGeometry::Request& req,
    mesh_msgs::GetGeometry::Response& res)
{
    ROS_INFO("Get geometry vertexnormals");
    lvr2::PlutoMapIO pmio(inputFile);

    // Vertex normals
    auto vertexNormals = pmio.getVertexNormals();
    unsigned int nVertexNormals = vertexNormals.size() / 3;
    ROS_INFO_STREAM("Found " << nVertexNormals << " vertex normals");
    res.mesh_geometry_stamped.mesh_geometry.vertex_normals.resize(nVertexNormals);
    for (unsigned int i = 0; i < nVertexNormals; i++)
    {
        res.mesh_geometry_stamped.mesh_geometry.vertex_normals[i].x = vertexNormals[i * 3];
        res.mesh_geometry_stamped.mesh_geometry.vertex_normals[i].y = vertexNormals[i * 3 + 1];
        res.mesh_geometry_stamped.mesh_geometry.vertex_normals[i].z = vertexNormals[i * 3 + 2];
    }

    // Header
    res.mesh_geometry_stamped.uuid = mesh_uuid;
    res.mesh_geometry_stamped.header.frame_id = "map";
    res.mesh_geometry_stamped.header.stamp = ros::Time::now();

    ROS_INFO("Done");

    return true;
}


bool hdf5_to_msg::service_getVertexColors(
    mesh_msgs::GetVertexColors::Request& req,
    mesh_msgs::GetVertexColors::Response& res)
{
    ROS_INFO("Get vertex colors");
    lvr2::PlutoMapIO pmio(inputFile);

    // Vertex colors
    auto vertexColors = pmio.getVertexColors();
    unsigned int nVertices = vertexColors.size() / 3;
    ROS_INFO_STREAM("Found " << nVertices << " vertices for vertex colors");
    res.mesh_vertex_colors_stamped.mesh_vertex_colors.vertex_colors.resize(nVertices);
    for (unsigned int i = 0; i < nVertices; i++)
    {
        res.mesh_vertex_colors_stamped.mesh_vertex_colors
            .vertex_colors[i].r = vertexColors[i * 3] / 255.0f;
        res.mesh_vertex_colors_stamped.mesh_vertex_colors
            .vertex_colors[i].g = vertexColors[i * 3 + 1] / 255.0f;
        res.mesh_vertex_colors_stamped.mesh_vertex_colors
            .vertex_colors[i].b = vertexColors[i * 3 + 2] / 255.0f;
        res.mesh_vertex_colors_stamped.mesh_vertex_colors
            .vertex_colors[i].a = 1;
    }

    // Header
    res.mesh_vertex_colors_stamped.uuid = mesh_uuid;
    res.mesh_vertex_colors_stamped.header.frame_id = "map";
    res.mesh_vertex_colors_stamped.header.stamp = ros::Time::now();

    ROS_INFO("Done");

    return true;
}

bool hdf5_to_msg::service_getMaterials(
    mesh_msgs::GetMaterials::Request& req,
    mesh_msgs::GetMaterials::Response& res)
{
    ROS_INFO("Get materials");
    lvr2::PlutoMapIO pmio(inputFile);


    // Materials
    auto materials = pmio.getMaterials();
    auto materialFaceIndices = pmio.getMaterialFaceIndices(); // for each face: material index
    unsigned int nMaterials = materials.size();
    unsigned int nFaces = materialFaceIndices.size();
    ROS_INFO_STREAM("Found " << nMaterials << " materials and " << nFaces << " faces");
    res.mesh_materials_stamped.mesh_materials.materials.resize(nMaterials);
    for (uint32_t i = 0; i < nMaterials; i++)
    {
        int texture_index = materials[i].textureIndex;

        // has texture
        res.mesh_materials_stamped
            .mesh_materials
            .materials[i]
            .has_texture = texture_index >= 0;

        // texture index
        res.mesh_materials_stamped.mesh_materials.materials[i]
            .texture_index = static_cast<uint32_t>(texture_index);
        // color
        res.mesh_materials_stamped.mesh_materials.materials[i]
            .color.r = materials[i].r / 255.0f;
        res.mesh_materials_stamped.mesh_materials.materials[i]
            .color.g = materials[i].g / 255.0f;
        res.mesh_materials_stamped.mesh_materials.materials[i]
            .color.b = materials[i].b / 255.0f;
        res.mesh_materials_stamped.mesh_materials.materials[i]
            .color.a = 1;
    }

    // Clusters
    // Map materials to face IDs
    std::map<uint32_t, vector<uint32_t>> materialToFaces;
    // Iterate over face <> material index
    for (uint32_t i = 0; i < nFaces; i++)
    {
        // material index for face i
        uint32_t materialIndex = materialFaceIndices[i];

        if (materialToFaces.count(materialIndex) == 0)
        {
            materialToFaces.insert(std::make_pair(materialIndex, vector<uint32_t>()));
        }

        materialToFaces[materialIndex].push_back(i);
    }
    // For each material, map contains a list of faces
    res.mesh_materials_stamped.mesh_materials.clusters.resize(nMaterials);
    for (uint32_t i = 0; i < nMaterials; i++)
    {
        for (uint32_t j = 0; j < materialToFaces[i].size(); j++)
        {
            res.mesh_materials_stamped.mesh_materials.clusters[i].face_indices.push_back(materialToFaces[i][j]);
        }
    }
    res.mesh_materials_stamped.mesh_materials.cluster_materials.resize(nMaterials);
    for (uint32_t i = 0; i < nMaterials; i++)
    {
        res.mesh_materials_stamped.mesh_materials.cluster_materials[i] = i;
    }

    // Vertex Tex Coords
    auto vertexTexCoords = pmio.getVertexTextureCoords();
    unsigned int nVertices = vertexTexCoords.size() / 3;
    res.mesh_materials_stamped.mesh_materials.vertex_tex_coords.resize(nVertices);
    for (uint32_t i = 0; i < nVertices; i++)
    {
        // coords: u/v/w
        // w is always 0
        res.mesh_materials_stamped.mesh_materials.vertex_tex_coords[i].u = vertexTexCoords[3 * i];
        res.mesh_materials_stamped.mesh_materials.vertex_tex_coords[i].v = vertexTexCoords[3 * i + 1];
    }

    // Header
    res.mesh_materials_stamped.uuid = mesh_uuid;
    res.mesh_materials_stamped.header.frame_id = "map";
    res.mesh_materials_stamped.header.stamp = ros::Time::now();

    ROS_INFO("Done");

    return true;
}

bool hdf5_to_msg::service_getTexture(
    mesh_msgs::GetTexture::Request& req,
    mesh_msgs::GetTexture::Response& res)
{
    ROS_INFO("Get texture");
    lvr2::PlutoMapIO pmio(inputFile);

    for (auto texture : pmio.getTextures())
    {
        if (std::stoi(texture.name) == req.texture_index)
        {
            res.texture.texture_index = req.texture_index;
            res.texture.uuid = mesh_uuid;
            sensor_msgs::Image image;
            sensor_msgs::fillImage( // TODO: only RGB, breaks when using other color channels
                image,
                "rgb8",
                texture.height,
                texture.width,
                texture.width * 3, // step size
                texture.data.data()
            );

            res.texture.image = image;

            ROS_INFO("Done");

            return true;    
        }
    }

    return false;    
}

bool hdf5_to_msg::service_getLabeledClusters(
    mesh_msgs::GetLabeledClusters::Request& req,
    mesh_msgs::GetLabeledClusters::Response& res)
{
    ROS_INFO("Get labeled clusters");
    lvr2::PlutoMapIO pmio(inputFile);

    // iterate over groups
    auto groups = pmio.getLabelGroups();
    for (size_t i = 0; i < groups.size(); i++)
    {
        // iterate over labels in group
        auto labelsInGroup = pmio.getAllLabelsOfGroup(groups[i]);
        for (size_t j = 0; j < labelsInGroup.size(); j++)
        {
            // copy label
            auto faceIds = pmio.getFaceIdsOfLabel(groups[i], labelsInGroup[j]);
            mesh_msgs::Cluster cluster;
            std::stringstream ss;
            ss << groups[i] << "_" << labelsInGroup[j];
            cluster.label = ss.str();
            cluster.face_indices.resize(faceIds.size());
            for (size_t k = 0; k < faceIds.size(); k++)
            {
                cluster.face_indices[k] = faceIds[k];
            }
            res.clusters.push_back(cluster);
        }
    }

    return true;
}

bool hdf5_to_msg::service_getLabelGroups(
    label_manager::GetLabelGroups::Request& req,
    label_manager::GetLabelGroups::Response& res)
{
    ROS_INFO("Get label groups");
    // TODO
    ROS_ERROR("Not implemented");

    return false;
}

bool hdf5_to_msg::service_getLabeledClusterGroup(
    label_manager::GetLabeledClusterGroup::Request& req,
    label_manager::GetLabeledClusterGroup::Response& res)
{
    ROS_INFO("Get labeled cluster group");
    // TODO
    ROS_ERROR("Not implemented");

    return false;
}

bool hdf5_to_msg::service_deleteLabel(
    label_manager::DeleteLabel::Request& req,
    label_manager::DeleteLabel::Response& res)
{
    ROS_INFO("Delete label");
    // TODO
    ROS_ERROR("Not implemented");

    return false;
}

bool hdf5_to_msg::service_getRoughness(
    mesh_msgs::GetVertexCosts::Request& req,
    mesh_msgs::GetVertexCosts::Response& res)
{
    ROS_INFO("Get roughness");

    lvr2::PlutoMapIO pmio(inputFile);
    auto roughness = pmio.getRoughness();

    res.mesh_vertex_costs_stamped.mesh_vertex_costs.costs.resize(roughness.size());
    for (uint32_t i = 0; i < roughness.size(); i++)
    {
        res.mesh_vertex_costs_stamped.mesh_vertex_costs.costs[i] = roughness[i];
    }

    res.mesh_vertex_costs_stamped.uuid = mesh_uuid;
    res.mesh_vertex_costs_stamped.type = "roughness";

    return true;
}

bool hdf5_to_msg::service_getHeightDifference(
    mesh_msgs::GetVertexCosts::Request& req,
    mesh_msgs::GetVertexCosts::Response& res)
{
    ROS_INFO("Get height difference");

    lvr2::PlutoMapIO pmio(inputFile);
    auto heightDiff = pmio.getRoughness();

    res.mesh_vertex_costs_stamped.mesh_vertex_costs.costs.resize(heightDiff.size());
    for (uint32_t i = 0; i < heightDiff.size(); i++)
    {
        res.mesh_vertex_costs_stamped.mesh_vertex_costs.costs[i] = heightDiff[i];
    }

    res.mesh_vertex_costs_stamped.uuid = mesh_uuid;
    res.mesh_vertex_costs_stamped.type = "height_difference";

    return true;
}

void hdf5_to_msg::callback_clusterLabel(const mesh_msgs::ClusterLabel::ConstPtr& msg)
{
    ROS_INFO_STREAM("New cluster label for mesh: " << msg->uuid);

    if (msg->uuid.compare(mesh_uuid) != 0)
    {
        ROS_ERROR("Invalid mesh UUID");
        return;
    }

    lvr2::PlutoMapIO pmio(inputFile);

    // TODO: implement optional override
    ROS_WARN("Override is enabled by default");

    // split label id into group and name
    std::vector<std::string> split_results;
    boost::split(split_results, msg->cluster.label, [](char c){ return c == '_'; });

    if (split_results.size() != 2)
    {
        ROS_ERROR("Received illegal cluster name");
        return;
    }


    std::string label_group = split_results[0];
    std::string label_name = split_results[1];
    vector<uint32_t> indices;
    for (size_t i = 0; i < msg->cluster.face_indices.size(); i++)
    {
        indices.push_back(msg->cluster.face_indices[i]);
    }

    // write to hdf5
    pmio.addLabel(label_group, label_name, indices);
}

// ===================================================================================================================

}

int main(int argc, char **args)
{
    ROS_INFO("Running ...");
    ros::init(argc, args, "hdf5_to_msg");
    lvr_ros::hdf5_to_msg hdf5_to_msg;
    ros::MultiThreadedSpinner spinner(4); // Use 4 threads
    spinner.spin(); // spin() will not return until the node has been shutdown
    return 0;
}
