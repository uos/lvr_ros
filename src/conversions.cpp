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
 * created on: 30.04.2014
 *
 * conversions.cpp
 *
 * Author: Sebastian Pütz <spuetz@uos.de>,
 *         Henning Deeken <hdeeken@uos.de>,
 *         Marcel Mrozinski <mmrozins@uos.de>,
 *         Tristan Igelbrink <tigelbri@uos.de>
 *
 */

#include "lvr_ros/conversions.h"
#include "lvr_ros/colors.h"
#include <cmath>

namespace lvr_ros
{

bool fromMeshBufferToMeshGeometryMessage(
    const lvr2::MeshBufferPtr<Vec>& buffer,
    mesh_msgs::MeshGeometry& mesh_geometry
){
    unsigned int n_vertices = buffer->getVertices().size() / 3;
    unsigned int n_faces = buffer->getFaceIndices().size() / 3;

    ROS_DEBUG_STREAM("Copy vertices from MeshBuffer to MeshGeometry.");

    // Copy vertices
    mesh_geometry.vertices.resize(n_vertices);
    auto buffer_vertices = buffer->getVertices();
    for (unsigned int i = 0; i < n_vertices; i++)
    {
        mesh_geometry.vertices[i].x = buffer_vertices[i * 3];
        mesh_geometry.vertices[i].y = buffer_vertices[i * 3 + 1];
        mesh_geometry.vertices[i].z = buffer_vertices[i * 3 + 2];
    }
    buffer_vertices.clear();

    ROS_DEBUG_STREAM("Copy faces from MeshBuffer to MeshGeometry.");

    // Copy faces
    auto buffer_faces = buffer->getFaceIndices();
    mesh_geometry.faces.resize(n_faces);
    for (unsigned int i = 0; i < n_faces; i++)
    {
        mesh_geometry.faces[i].vertex_indices[0] = buffer_faces[i * 3];
        mesh_geometry.faces[i].vertex_indices[1] = buffer_faces[i * 3 + 1];
        mesh_geometry.faces[i].vertex_indices[2] = buffer_faces[i * 3 + 2];
    }
    buffer_faces.clear();


    // Copy vertex normals
    auto buffer_vertexnormals = buffer->getVertexNormals();
    if(!buffer_vertexnormals.empty())
    {
        if(buffer_vertexnormals.size() != n_vertices * 3)
        {
            ROS_FATAL_STREAM("The number of normals in the MeshBuffer differs"
                                 "from the number of vertices! Ignoring normals!");
        }else{
            ROS_DEBUG_STREAM("Copy normals from MeshBuffer to MeshGeometry.");

            mesh_geometry.vertex_normals.resize(n_vertices);
            for (unsigned int i = 0; i < n_vertices; i++) {
                mesh_geometry.vertex_normals[i].x = buffer_vertexnormals[i * 3];
                mesh_geometry.vertex_normals[i].y = buffer_vertexnormals[i * 3 + 1];
                mesh_geometry.vertex_normals[i].z = buffer_vertexnormals[i * 3 + 2];
            }
            buffer_vertexnormals.clear();
        }
    }else{
        ROS_DEBUG_STREAM("No vertex normals given!");
    }

    ROS_DEBUG_STREAM("Successfully copied the MeshBuffer "
                         "geometry to the MeshGeometry message.");
    return true;
}

bool fromMeshBufferToMeshMessages(
    const lvr2::MeshBufferPtr<Vec>& buffer,
    mesh_msgs::MeshGeometry& mesh_geometry,
    mesh_msgs::MeshMaterials& mesh_materials,
    mesh_msgs::MeshVertexColors& mesh_vertex_colors,
    boost::optional<std::vector<mesh_msgs::Texture>&> texture_cache,
    std::string mesh_uuid
)
{
    unsigned int n_vertices = buffer->getVertices().size() / 3;
    unsigned int n_faces = buffer->getFaceIndices().size() / 3;
    unsigned int n_clusters = buffer->getClusterFaceIndices().size();
    unsigned int n_materials = buffer->getMaterials().size();
    unsigned int n_textures = buffer->getTextures().size();
    unsigned int n_vertex_colors = buffer->getVertexColors().size();

    // copy vertices, faces and normals
    fromMeshBufferToMeshGeometryMessage(buffer, mesh_geometry);

    // Copy clusters
    auto buffer_clusters = buffer->getClusterFaceIndices();
    mesh_materials.clusters.resize(n_clusters);
    for (unsigned int i = 0; i < n_clusters; i++)
    {
        int n = buffer_clusters[i].size();
        mesh_materials.clusters[i].face_indices.resize(n);
        for (unsigned int j = 0; j < n; j++)
        {
            mesh_materials.clusters[i].face_indices[j] = buffer_clusters[i][j];
        }
    }
    buffer_clusters.clear();

    // Copy materials
    auto buffer_materials = buffer->getMaterials();
    mesh_materials.materials.resize(n_materials);
    for (unsigned int i = 0; i < n_materials; i++)
    {
        lvr2::Material m = buffer_materials[i];
        if (m.m_color)
        {
            mesh_materials.materials[i].color.r = m.m_color.get()[0]/255.0;
            mesh_materials.materials[i].color.g = m.m_color.get()[1]/255.0;
            mesh_materials.materials[i].color.b = m.m_color.get()[2]/255.0;
            mesh_materials.materials[i].color.a = 1.0;
        }
        else
        {
            mesh_materials.materials[i].color.r = 1.0;
            mesh_materials.materials[i].color.g = 1.0;
            mesh_materials.materials[i].color.b = 1.0;
            mesh_materials.materials[i].color.a = 1.0;
        }
        if (m.m_texture)
        {
            mesh_materials.materials[i].has_texture = true;
            mesh_materials.materials[i].texture_index = (int)m.m_texture.get().idx();
        }
        else
        {
            mesh_materials.materials[i].has_texture = false;
            mesh_materials.materials[i].texture_index = 0;
        }
    }
    buffer_materials.clear();

    // Copy cluster material indices
    auto buffer_cluster_materials = buffer->getClusterMaterialIndices();
    mesh_materials.cluster_materials.resize(n_clusters);
    for (unsigned int i = 0; i < n_clusters; i++)
    {
        mesh_materials.cluster_materials[i] = buffer_cluster_materials[i];
    }
    buffer_cluster_materials.clear();

    // Copy vertex tex coords
    auto buffer_texcoords = buffer->getVertexTextureCoordinates();
    if (buffer_texcoords.size() > 0)
    {
        mesh_materials.vertex_tex_coords.resize(n_vertices);

        for (unsigned int i = 0; i < n_vertices; i++)
        {
            mesh_materials.vertex_tex_coords[i].u = buffer_texcoords[i * 3];
            mesh_materials.vertex_tex_coords[i].v = buffer_texcoords[i * 3 + 1];
        }

        buffer_texcoords.clear();
    }

    // Copy vertex colors
    if (n_vertex_colors > 0)
    {
        auto buffer_vertex_colors = buffer->getVertexColors();
        mesh_vertex_colors.vertex_colors.resize(n_vertices);
        for (unsigned int i = 0; i < n_vertices; i++)
        {
            float r = buffer_vertex_colors[i * 3]/255.0;
            mesh_vertex_colors.vertex_colors[i].r = r;
            mesh_vertex_colors.vertex_colors[i].g = buffer_vertex_colors[i * 3 + 1]/255.0;
            mesh_vertex_colors.vertex_colors[i].b = buffer_vertex_colors[i * 3 + 2]/255.0;
            mesh_vertex_colors.vertex_colors[i].a = 1.0;
        }
        buffer_vertex_colors.clear();
    }

    // If texture cache is available, cache textures in given vector
    if (texture_cache)
    {
        auto buffer_textures = buffer->getTextures();
        texture_cache.get().resize(n_textures);
        for (unsigned int i = 0; i < n_textures; i++)
        {
            sensor_msgs::Image image;
            sensor_msgs::fillImage(
                image,
                "rgb8",
                buffer_textures[i].m_height,
                buffer_textures[i].m_width,
                buffer_textures[i].m_width * 3, // step size
                buffer_textures[i].m_data
            );
            mesh_msgs::Texture texture;
            texture.uuid = mesh_uuid;
            texture.texture_index = i;
            texture.image = image;
            texture_cache.get().at(i) = texture;
        }
        buffer_textures.clear();
    }

    return true;
}

bool fromMeshBufferToTriangleMesh(
    const lvr::MeshBufferPtr& buffer,
    mesh_msgs::TriangleMesh& mesh)
{
    return fromMeshBufferToTriangleMesh(*buffer, mesh);
}

bool fromMeshBufferToTriangleMesh(
    lvr::MeshBuffer& buffer,
    mesh_msgs::TriangleMesh& mesh)
{
    size_t numVertices = 0;
    size_t numFaces = 0;
    size_t numNormals = 0;
    lvr::coord3fArr verticesArray = buffer.getIndexedVertexArray(numVertices);
    lvr::coord3fArr normalsArray = buffer.getIndexedVertexNormalArray(numNormals);
    lvr::uintArr facesArray = buffer.getFaceArray(numFaces);

    ROS_DEBUG_STREAM("number vertices: " << numVertices);
    ROS_DEBUG_STREAM("number triangles: " << numFaces);
    ROS_DEBUG_STREAM("number normals: " << numNormals);


    mesh.vertices.resize(numVertices);
    mesh.vertex_normals.resize(numNormals);
    mesh.triangles.resize(numFaces);

    if (numVertices && numFaces)
    {
        // copy vertices
        for (unsigned int i = 0; i < numVertices; i++)
        {
            mesh.vertices[i].x = verticesArray[i].x;
            mesh.vertices[i].y = verticesArray[i].y;
            mesh.vertices[i].z = verticesArray[i].z;
        }

        // copy triangles
        for (unsigned int i = 0; i < numFaces; i++)
        {
            mesh.triangles[i].vertex_indices[0] = facesArray[i * 3];
            mesh.triangles[i].vertex_indices[1] = facesArray[i * 3 + 1];
            mesh.triangles[i].vertex_indices[2] = facesArray[i * 3 + 2];
        }

        // copy point normals
        for (unsigned int i = 0; i < numNormals; i++)
        {
            mesh.vertex_normals[i].x = normalsArray[i].x;
            mesh.vertex_normals[i].y = normalsArray[i].y;
            mesh.vertex_normals[i].z = normalsArray[i].z;
        }

        /*
              optional:
              geometry_msgs/Point[] vertex_normals
              std_msgs/ColorRGBA[] vertex_colors
              geometry_msgs/Point[] vertex_texture_coords
              mesh_msgs/Material[] face_materials
              sensor_msgs/Image[] textures
              mesh_msgs/Cluster[] clusters
        */

        return true;
    }
    else
    {
        return false;
    }
}

bool fromMeshGeometryToMeshBuffer(
    const mesh_msgs::MeshGeometryConstPtr& mesh_geometry_ptr,
    lvr2::MeshBuffer<Vec>& buffer)
{
    fromMeshGeometryToMeshBuffer(*mesh_geometry_ptr, buffer);
}

bool fromMeshGeometryToMeshBuffer(
    const mesh_msgs::MeshGeometryConstPtr& mesh_geometry_ptr,
    lvr2::MeshBufferPtr<Vec>& buffer_ptr)
{
    fromMeshGeometryToMeshBuffer(*mesh_geometry_ptr, *buffer_ptr);
}

bool fromMeshGeometryToMeshBuffer(
    const mesh_msgs::MeshGeometryPtr& mesh_geometry_ptr,
    lvr2::MeshBufferPtr<Vec>& buffer_ptr)
{
    fromMeshGeometryToMeshBuffer(*mesh_geometry_ptr, *buffer_ptr);
}

bool fromMeshGeometryToMeshBuffer(
    const mesh_msgs::MeshGeometryPtr& mesh_geometry_ptr,
    lvr2::MeshBuffer<Vec>& buffer)
{
    fromMeshGeometryToMeshBuffer(*mesh_geometry_ptr, buffer);
}

bool fromMeshGeometryToMeshBuffer(
    const mesh_msgs::MeshGeometry& mesh_geometry,
    lvr2::MeshBufferPtr<Vec>& buffer_ptr)
{
    fromMeshGeometryToMeshBuffer(mesh_geometry, *buffer_ptr);
}

bool fromMeshGeometryToMeshBuffer(
    const mesh_msgs::MeshGeometry& mesh_geometry,
    lvr2::MeshBuffer<Vec>& buffer)
{
    std::vector<float> vertices;
    vertices.reserve(mesh_geometry.vertices.size() * 3);
    for(const geometry_msgs::Point& p : mesh_geometry.vertices)
    {
        vertices.push_back(p.x);
        vertices.push_back(p.y);
        vertices.push_back(p.z);
    }
    buffer.setVertices(vertices);

    std::vector<unsigned int> faces;
    faces.reserve(mesh_geometry.faces.size() * 3);
    for(const mesh_msgs::TriangleIndices& t : mesh_geometry.faces)
    {
        faces.push_back(t.vertex_indices[0]);
        faces.push_back(t.vertex_indices[1]);
        faces.push_back(t.vertex_indices[2]);
    }
    buffer.setFaceIndices(faces);

    std::vector<float> normals;
    normals.reserve(mesh_geometry.vertex_normals.size() * 3);
    for(const geometry_msgs::Point& n : mesh_geometry.vertex_normals)
    {
        normals.push_back(n.x);
        normals.push_back(n.y);
        normals.push_back(n.z);
    }
    buffer.setVertexNormals(normals);
    return true;
}

bool fromTriangleMeshToMeshBuffer(
    const mesh_msgs::TriangleMesh& mesh,
    lvr::MeshBuffer& buffer)
{
    // copy vertices
    vector<float> vertices;
    for (unsigned int i = 0; i < mesh.vertices.size(); i++)
    {
        vertices.push_back((float) mesh.vertices[i].x);
        vertices.push_back((float) mesh.vertices[i].y);
        vertices.push_back((float) mesh.vertices[i].z);
    }
    buffer.setVertexArray(vertices);

    // copy faces
    vector<unsigned int> faces;
    for (unsigned int i = 0; i < mesh.triangles.size(); i++)
    {
        faces.push_back((unsigned int) mesh.triangles[i].vertex_indices[0]);
        faces.push_back((unsigned int) mesh.triangles[i].vertex_indices[1]);
        faces.push_back((unsigned int) mesh.triangles[i].vertex_indices[2]);
    }
    buffer.setFaceArray(faces);

    // copy normals
    vector<float> normals;
    for (unsigned int i = 0; i < mesh.vertex_normals.size(); i++)
    {
        normals.push_back((float) mesh.vertex_normals[i].x);
        normals.push_back((float) mesh.vertex_normals[i].y);
        normals.push_back((float) mesh.vertex_normals[i].z);
    }
    buffer.setVertexNormalArray(normals);

    return true;
}

bool readMeshBuffer(lvr::MeshBufferPtr& buffer, string path)
{
    lvr::ModelFactory io_factory;
    lvr::ModelPtr model = io_factory.readModel(path);

    if (!model)
    {
        return false;
    }
    else
    {
        buffer = model->m_mesh;
        return true;
    }
}

bool writeMeshBuffer(lvr::MeshBufferPtr& buffer, string path)
{
    lvr::ModelPtr model(new lvr::Model(buffer));
    lvr::ModelFactory::saveModel(model, path);
    return true;
}

bool readTriangleMesh(mesh_msgs::TriangleMesh& mesh, string path)
{
    lvr::ModelFactory io_factory;
    lvr::ModelPtr model = io_factory.readModel(path);

    if (!model)
    {
        return false;
    }
    return fromMeshBufferToTriangleMesh(model->m_mesh, mesh);
}

bool writeTriangleMesh(mesh_msgs::TriangleMesh& mesh, string path)
{
    lvr::MeshBuffer buffer;
    if (fromTriangleMeshToMeshBuffer(mesh, buffer))
    {
        lvr::MeshBufferPtr buffer_ptr = boost::make_shared<lvr::MeshBuffer>(buffer);

        lvr::ModelPtr model(new lvr::Model(buffer_ptr));
        lvr::ModelFactory::saveModel(model, path);
        return true;
    }
    else return false;
}

void removeDuplicates(lvr::MeshBuffer& buffer)
{
    lvr::floatArr old_vertexBuffer;
    lvr::uintArr old_indexBuffer;
    std::vector<float> new_vertexBuffer;
    std::vector<unsigned int> new_indexBuffer;

    size_t old_numVertices, old_numIndices;
    size_t new_numVertices, new_numIndices;

    old_vertexBuffer = buffer.getVertexArray(old_numVertices);
    old_indexBuffer = buffer.getFaceArray(old_numIndices);

    std::map<lvr::Vertex<float>, unsigned int> vertexMap;
    size_t pos;
    int index;

    for (int i = 0; i < old_numIndices; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            index = old_indexBuffer[3 * i + j];

            lvr::Vertex<float> vertex =
                lvr::Vertex<float>(old_vertexBuffer[3 * index],
                                   old_vertexBuffer[3 * index + 1],
                                   old_vertexBuffer[3 * index + 2]);

            if (vertexMap.find(vertex) != vertexMap.end())
            {
                pos = vertexMap[vertex];
            }
            else
            {
                pos = new_vertexBuffer.size() / 3;
                new_vertexBuffer.push_back(vertex[0]);
                new_vertexBuffer.push_back(vertex[1]);
                new_vertexBuffer.push_back(vertex[2]);

                vertexMap.insert(pair<lvr::Vertex<float>, unsigned int>(vertex, pos));
            }

            new_indexBuffer.push_back(pos);
        }
    }
    buffer.setVertexArray(new_vertexBuffer);
    buffer.setFaceArray(new_indexBuffer);
}

void intensityToTriangleRainbowColors(
    const std::vector<float>& intensity,
    mesh_msgs::TriangleMesh& mesh,
    float min,
    float max
)
{
    float range = max - min;

    float r, g, b;
    float a = 1;

    for (size_t i = 0; i < intensity.size(); i++)
    {
        float norm = (intensity[i] - min) / range;
        getRainbowColor(norm, r, g, b);
        std_msgs::ColorRGBA color;
        color.a = a;
        color.r = r;
        color.g = g;
        color.b = b;
        mesh.triangle_colors.push_back(color);
    }
}

void intensityToTriangleRainbowColors(const std::vector<float>& intensity, mesh_msgs::TriangleMesh& mesh)
{
    float min = std::numeric_limits<float>::max();
    float max = std::numeric_limits<float>::min();

    for (size_t i = 0; i < intensity.size(); i++)
    {
        if (!std::isfinite(intensity[i])) continue;
        if (min > intensity[i]) min = intensity[i];
        if (max < intensity[i]) max = intensity[i];
    }
    intensityToTriangleRainbowColors(intensity, mesh, min, max);
}

void intensityToVertexRainbowColors(
    const std::vector<float>& intensity,
    mesh_msgs::TriangleMesh& mesh,
    float min,
    float max
)
{
    float range = max - min;

    float r, g, b;
    float a = 1;

    for (size_t i = 0; i < intensity.size(); i++)
    {
        float norm = (intensity[i] - min) / range;
        getRainbowColor(norm, r, g, b);
        std_msgs::ColorRGBA color;
        color.a = a;
        color.r = r;
        color.g = g;
        color.b = b;
        mesh.vertex_colors.push_back(color);
    }
}

void intensityToVertexRainbowColors(
    const lvr2::DenseVertexMap<float>& intensity,
    mesh_msgs::TriangleMesh& mesh,
    float min,
    float max
)
{
    float range = max - min;

    float r, g, b;
    float a = 1;

    for (auto intensity_val: intensity)
    {
        float norm = (intensity[intensity_val] - min) / range;
        getRainbowColor(norm, r, g, b);
        std_msgs::ColorRGBA color;
        color.a = a;
        color.r = r;
        color.g = g;
        color.b = b;
        mesh.triangle_colors.push_back(color);
    }
}

void intensityToVertexRainbowColors(const std::vector<float>& intensity, mesh_msgs::TriangleMesh& mesh)
{
    float min = std::numeric_limits<float>::max();
    float max = std::numeric_limits<float>::min();

    for (size_t i = 0; i < intensity.size(); i++)
    {
        if (!std::isfinite(intensity[i])) continue;
        if (min > intensity[i]) min = intensity[i];
        if (max < intensity[i]) max = intensity[i];
    }
    intensityToVertexRainbowColors(intensity, mesh, min, max);
}

void removeDuplicates(mesh_msgs::TriangleMesh& mesh)
{
    lvr::MeshBuffer buffer;
    fromTriangleMeshToMeshBuffer(mesh, buffer);
    removeDuplicates(buffer);

    lvr::MeshBufferPtr buffer_ptr = boost::make_shared<lvr::MeshBuffer>(buffer);
    fromMeshBufferToTriangleMesh(buffer_ptr, mesh);
}

static inline bool hasCloudChannel(const sensor_msgs::PointCloud2& cloud, const std::string& field_name)
{
    // Get the index we need
    for (size_t d = 0; d < cloud.fields.size(); ++d)
        if (cloud.fields[d].name == field_name)
            return true;
    return false;
}

bool fromPointCloud2ToPointBuffer(const sensor_msgs::PointCloud2& cloud, PointBuffer& buffer)
{
    ROS_DEBUG_STREAM("convert from PointCloud2 to PointBuffer.");

    size_t size = cloud.height * cloud.width;

    typedef sensor_msgs::PointCloud2ConstIterator<float> CloudIterFloat;
    typedef sensor_msgs::PointCloud2ConstIterator <uint8_t> CloudIterUInt8;


    std::list<int> filter_nan;
    lvr::PointBuffer old_buffer;


    // copy point data
    CloudIterFloat iter_x_filter(cloud, "x");
    CloudIterFloat iter_y_filter(cloud, "y");
    CloudIterFloat iter_z_filter(cloud, "z");

    // size without NaN values
    size = 0;
    for (int i = 0; iter_x_filter != iter_x_filter.end();
         ++iter_x_filter, ++iter_y_filter, ++iter_z_filter, i++)
    {
        if (!std::isnan(*iter_x_filter) && !std::isnan(*iter_y_filter) && !std::isnan(*iter_z_filter))
            size++;
        else
        {
            filter_nan.push_back(i);
        }
    }

    filter_nan.sort();

    float *pointData = new float[size * 3];

    // copy point data
    CloudIterFloat iter_x(cloud, "x");
    CloudIterFloat iter_y(cloud, "y");
    CloudIterFloat iter_z(cloud, "z");


    std::list<int> tmp_filter = filter_nan;
    int index = 0;
    for (int i = 0;
         iter_x != iter_x.end();
         ++iter_x, ++iter_y, ++iter_z,
             index++)
    {
        // skip NaN point values
        if (!tmp_filter.empty() && index == tmp_filter.front())
        {
            tmp_filter.pop_front();
            continue;
        }

        // copy point
        pointData[i] = *iter_x;
        pointData[i + 1] = *iter_y;
        pointData[i + 2] = *iter_z;

        i += 3;

    }
    old_buffer.setPointArray(lvr::floatArr(pointData), size);


    // copy point normals if available
    bool normalsAvailable =
        hasCloudChannel(cloud, "normal_x")
        && hasCloudChannel(cloud, "normal_y")
        && hasCloudChannel(cloud, "normal_z");

    if (normalsAvailable)
    {
        ROS_DEBUG_STREAM("include normals in conversion.");
        CloudIterFloat iter_n_x(cloud, "normal_x");
        CloudIterFloat iter_n_y(cloud, "normal_y");
        CloudIterFloat iter_n_z(cloud, "normal_z");
        float *normalsData = new float[size * 3];
        tmp_filter = filter_nan;
        int index = 0;
        for (int i = 0;
             iter_n_x != iter_n_x.end();
             ++iter_n_x, ++iter_n_y, ++iter_n_z,
                 index++)
        {

            // skip NaN point values
            if (!tmp_filter.empty() && index == tmp_filter.front())
            {
                tmp_filter.pop_front();
                continue;
            }

            // copy normal
            normalsData[i] = *iter_n_x;
            normalsData[i + 1] = *iter_n_y;
            normalsData[i + 2] = *iter_n_z;

            i += 3;
        }
        old_buffer.setPointNormalArray(lvr::floatArr(normalsData), size);
    }


    // copy color data if available
    if (hasCloudChannel(cloud, "rgb"))
    {
        ROS_DEBUG_STREAM("include rgb in conversion.");
        CloudIterUInt8 iter_rgb(cloud, "rgb");
        uint8_t *colorData = new uint8_t[size * 3];
        tmp_filter = filter_nan;
        int index = 0;
        for (int i = 0; iter_rgb != iter_rgb.end();
             ++iter_rgb, index++)
        {

            // skip NaN point values
            if (!tmp_filter.empty() && index == tmp_filter.front())
            {
                tmp_filter.pop_front();
                continue;
            }

            // copy color rgb
            colorData[i] = iter_rgb[0];
            colorData[i + 1] = iter_rgb[1];
            colorData[i + 2] = iter_rgb[2];

            i += 3;
        }
        old_buffer.setPointColorArray(lvr::ucharArr(colorData), size);
    }


    // copy intensity if available
    if (hasCloudChannel(cloud, "intensities"))
    {
        ROS_DEBUG_STREAM("include intensities in conversion.");
        CloudIterFloat iter_int(cloud, "intensities");
        float *intensityData = new float[size];
        tmp_filter = filter_nan;
        int index = 0;
        for (int i = 0; iter_int != iter_int.end();
             ++iter_int, index++)
        {

            // skip NaN point values
            if (!tmp_filter.empty() && index == tmp_filter.front())
            {
                tmp_filter.pop_front();
                continue;
            }

            // copy intensity
            intensityData[i] = *iter_int;
            i++;
        }
        old_buffer.setPointIntensityArray(lvr::floatArr(intensityData), size);
    }
    buffer = PointBuffer(old_buffer);
    ROS_DEBUG_STREAM("conversion finished.");
    return true;
}

bool fromMeshGeometryMessageToMeshBuffer(
    const mesh_msgs::MeshGeometry& mesh_geometry,
    const lvr2::MeshBufferPtr<Vec>& buffer
)
{
    // copy vertices
    vector<float> vertices;
    vertices.reserve(mesh_geometry.vertices.size() * 3);
    for (auto vertex : mesh_geometry.vertices)
    {
        vertices.push_back(static_cast<float>(vertex.x));
        vertices.push_back(static_cast<float>(vertex.y));
        vertices.push_back(static_cast<float>(vertex.z));
    }
    buffer->setVertices(vertices);

    // copy faces
    vector<unsigned int> faces;
    faces.reserve(mesh_geometry.faces.size() * 3);
    for (auto face : mesh_geometry.faces)
    {
        faces.push_back(face.vertex_indices[0]);
        faces.push_back(face.vertex_indices[1]);
        faces.push_back(face.vertex_indices[2]);
    }
    buffer->setFaceIndices(faces);

    // copy normals
    vector<float> normals;
    normals.reserve(mesh_geometry.vertex_normals.size() * 3);
    for (auto normal : mesh_geometry.vertex_normals)
    {
        normals.push_back(static_cast<float>(normal.x));
        normals.push_back(static_cast<float>(normal.y));
        normals.push_back(static_cast<float>(normal.z));
    }
    buffer->setVertexNormals(normals);

    return true;
}
} // end namespace
