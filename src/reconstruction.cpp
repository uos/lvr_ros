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
 * reconstruction.cpp
 *
 * Author: Sebastian Pütz <spuetz@uos.de>,
 *
 */

#include <iostream>
#include <memory>

using std::make_shared;

#include "lvr_ros/reconstruction.h"
#include "lvr_ros/conversions.h"

#include <lvr/io/PLYIO.hpp>
#include <lvr/config/lvropenmp.hpp>
#include <lvr/geometry/Matrix4.hpp>
#include <lvr/texture/Texture.hpp>
#include <lvr/texture/Transform.hpp>
#include <lvr/texture/Texturizer.hpp>
#include <lvr/texture/Statistics.hpp>
#include <lvr/geometry/QuadricVertexCosts.hpp>

#include <lvr2/geometry/HalfEdgeMesh.hpp>
#include <lvr2/geometry/Vector.hpp>
#include <lvr2/geometry/Point.hpp>
#include <lvr2/geometry/Normal.hpp>
#include <lvr2/algorithm/FinalizeAlgorithm.hpp>
#include <lvr2/geometry/BoundingBox.hpp>
#include <lvr2/algorithm/Planar.hpp>
#include <lvr2/algorithm/NormalAlgorithms.hpp>
#include <lvr2/algorithm/ClusterPainter.hpp>
#include <lvr2/geometry/Handles.hpp>
#include <lvr2/util/ClusterBiMap.hpp>

#include <lvr2/reconstruction/AdaptiveKSearchSurface.hpp>
#include <lvr2/reconstruction/BilinearFastBox.hpp>
#include <lvr2/reconstruction/FastReconstruction.hpp>
#include <lvr2/reconstruction/PointsetSurface.hpp>
#include <lvr2/reconstruction/SearchTree.hpp>
#include <lvr2/reconstruction/SearchTreeFlann.hpp>
#include <lvr2/reconstruction/HashGrid.hpp>
#include <lvr2/reconstruction/PointsetGrid.hpp>
#include <lvr2/io/PointBuffer.hpp>
#include <lvr2/util/Factories.hpp>
#include <lvr2/util/Panic.hpp>

namespace lvr_ros
{

Reconstruction::Reconstruction() : as_(node_handle, "reconstruction", boost::bind(&Reconstruction::reconstruct, this, _1), false)
{
    ros::NodeHandle nh("~");

    cloud_subscriber = node_handle.subscribe("/pointcloud", 1, &Reconstruction::pointCloudCallback, this);
    mesh_publisher = node_handle.advertise<mesh_msgs::TriangleMeshStamped>("/mesh", 1);

    // setup dynamic reconfigure
    reconfigure_server_ptr = DynReconfigureServerPtr(new DynReconfigureServer(nh));
    callback_type = boost::bind(&Reconstruction::reconfigureCallback, this, _1, _2);
    reconfigure_server_ptr->setCallback(callback_type);

    // start action server
    as_.start();
}

void Reconstruction::reconstruct(const lvr_ros::ReconstructGoalConstPtr& goal)
{
    try
    {
        lvr_ros::ReconstructResult result;
        createMesh(goal->point_cloud, result.mesh);
        as_.setSucceeded(result, "Published mesh.");
    }
    catch(std::exception& e)
    {
        ROS_ERROR_STREAM("Error: " << e.what());
        as_.setAborted();
    }
}

void Reconstruction::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud)
{
    mesh_msgs::TriangleMeshStamped mesh;
    createMesh(*cloud, mesh);
    mesh_publisher.publish(mesh);
}

void Reconstruction::reconfigureCallback(lvr_ros::ReconstructionConfig& config, uint32_t level)
{
    this->config = config;
}

bool Reconstruction::createMesh(const sensor_msgs::PointCloud2& cloud, mesh_msgs::TriangleMeshStamped& mesh_msg)
{
    PointBufferPtr point_buffer_ptr(new PointBuffer);
    lvr::MeshBufferPtr mesh_buffer_ptr(new lvr::MeshBuffer);

    if (!lvr_ros::fromPointCloud2ToPointBuffer(cloud, *point_buffer_ptr))
    {
        ROS_ERROR_STREAM("Could not convert point cloud from \"sensor_msgs::PointCloud2\" to \"lvr::PointBuffer\"!");
        return false;
    }
    if (!createMesh(point_buffer_ptr, mesh_buffer_ptr))
    {
        ROS_ERROR_STREAM("Reconstruction failed!");
        return false;
    }
    if (!lvr_ros::fromMeshBufferToTriangleMesh(mesh_buffer_ptr, mesh_msg.mesh))
    {
        ROS_ERROR_STREAM(
            "Could not convert point cloud from \"lvr::MeshBuffer\" to \"mesh_msgs::TriangleMeshStamped\"!");
        return false;
    }

    // setting header frame and stamp
    mesh_msg.header.frame_id = cloud.header.frame_id;
    mesh_msg.header.stamp = cloud.header.stamp;

    return true;
}

bool Reconstruction::createMesh(PointBufferPtr& point_buffer, lvr::MeshBufferPtr& mesh_buffer)
{
    // Create a point cloud manager
    string pcm_name = config.pcm;
    lvr2::PointsetSurfacePtr <Vec> surface;

    // Create point set surface object
    if (pcm_name == "PCL")
    {
        lvr2::panic("PCL not supported right meow!");
    }
    else if (
        pcm_name == "STANN" ||
        pcm_name == "FLANN" ||
        pcm_name == "NABO" ||
        pcm_name == "NANOFLANN"
        )
    {
        surface = make_shared < lvr2::AdaptiveKSearchSurface < Vec >> (
            point_buffer,
            pcm_name,
            config.kn,
            config.ki,
            config.kd,
            config.ransac
        );
    }
    else
    {
        ROS_ERROR_STREAM("Unable to create PointCloudManager.");
        ROS_ERROR_STREAM("Unknown option '" << pcm_name << "'.");
        ROS_ERROR_STREAM("Available PCMs are: ");
        ROS_ERROR_STREAM("STANN, STANN_RANSAC, PCL");
        return 0;
    }

    // Set search config for normal estimation and distance evaluation
    surface->setKd(config.kd);
    surface->setKi(config.ki);
    surface->setKn(config.kn);

    // Calculate normals if necessary
    if (!point_buffer->hasNormals() || config.recalcNormals)
    {
        surface->calculateSurfaceNormals();
    }
    else
    {
        ROS_INFO_STREAM("Using given normals.");
    }

    // Create an empty mesh
    lvr2::HalfEdgeMesh <Vec> mesh;

    // Determine whether to use intersections or voxelsize
    float resolution;
    bool useVoxelsize;
    if (config.intersections > 0)
    {
        resolution = config.intersections;
        useVoxelsize = false;
    }
    else
    {
        resolution = config.voxelsize;
        useVoxelsize = true;
    }

    // Create a point set grid for reconstruction
    string decomposition = config.decomposition;

    // Fail safe check
    if (decomposition != "MC" && decomposition != "PMC" && decomposition != "SF")
    {
        ROS_ERROR_STREAM("Unsupported decomposition type " << decomposition << ". Defaulting to PMC.");
        decomposition = "PMC";
    }

    shared_ptr <lvr2::GridBase> grid;
    unique_ptr <lvr2::FastReconstructionBase<Vec>> reconstruction;
    if (decomposition == "MC")
    {
        lvr2::panic("MC decomposition type not supported right now!");
    }
    else if (decomposition == "PMC")
    {
        lvr2::BilinearFastBox<Vec>::m_surface = surface;
        auto ps_grid = std::make_shared<lvr2::PointsetGrid<Vec, lvr2::BilinearFastBox<Vec>>>(
            resolution,
            surface,
            surface->getBoundingBox(),
            useVoxelsize,
            !config.noExtrusion
        );
        ps_grid->calcDistanceValues();
        grid = ps_grid;
        reconstruction = make_unique<lvr2::FastReconstruction<Vec, lvr2::BilinearFastBox<Vec>>>(ps_grid);
    }
    else if (decomposition == "SF")
    {
        lvr2::panic("SF decomposition type not supported right now!");
    }

    // Create mesh
    reconstruction->getMesh(mesh);

    auto faceNormals = calcFaceNormals(mesh);

    lvr2::ClusterBiMap <lvr2::FaceHandle> clusterSet;
    if (config.optimizePlanes)
    {
        clusterSet = iterativePlanarClusterGrowing(
            mesh,
            faceNormals,
            config.normalThreshold,
            config.planeIterations,
            config.minPlaneSize
        );
    }
    else
    {
        clusterSet = planarClusterGrowing(mesh, faceNormals, config.normalThreshold);
    }

    // Calc normals for vertices
    auto vertexNormals = calcVertexNormals(mesh, faceNormals, *surface);

    lvr2::FinalizeAlgorithm <Vec> finalize;
    finalize.setNormalData(vertexNormals);

    //if (colorMap)
    //{
    //    finalize.setColorData(*colorMap);
    //}

    mesh_buffer = finalize.apply(mesh);

    // // Create output model and save to file
    // auto model = new lvr::Model(buffer);
    // lvr::ModelPtr m(model);
    // cout << timestamp << "Saving mesh." << endl;
    // lvr::ModelFactory::saveModel( m, "triangle_mesh.ply");

    // // Save triangle mesh
    // if ( config.retesselate )
    // {
    //     mesh.finalizeAndRetesselate(config.generateTextures, config.lineFusionThreshold);
    // }
    // else
    // {
    //     mesh.finalize();
    // }

    // // Write classification to file
    // if ( config.writeClassificationResult )
    // {
    //     mesh.writeClassificationResult();
    // }

    // mesh_buffer = mesh.meshBuffer();

    ROS_INFO_STREAM("Reconstruction finished!");
    return true;
}

float *Reconstruction::getStatsCoeffs(std::string filename) const
{
    float *result = new float[14];
    std::ifstream in(filename.c_str());
    if (in.good())
    {
        for (int i = 0; i < 14; i++)
        {
            in >> result[i];
        }
        in.close();
    }
    else
    {
        for (int i = 0; i < 14; i++)
        {
            result[i] = 0.5;
        }
    }
    return result;
}


} /* namespace lvr_ros */


int main(int argc, char **args)
{
    ros::init(argc, args, "reconstruction");
    lvr_ros::Reconstruction reconstruction;
    ros::spin();

    return 0;
}
