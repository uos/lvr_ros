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
 * debug.cpp
 *
 * Author: Johan M. von Behren <johan@vonbehren.eu>
 *
 */

#include "lvr_ros/debug.h"
#include "lvr_ros/conversions.h"

namespace lvr_ros
{

Debug::Debug()
{
    ros::NodeHandle nh("~");

    cloud_subscriber = node_handle.subscribe("/pointcloud", 1, &Debug::pointCloudCallback, this);
}

Debug::~Debug()
{}

void Debug::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr cloud)
{
    dumpPointCloudToFile(cloud);
}

void Debug::dumpPointCloudToFile(const sensor_msgs::PointCloud2::ConstPtr cloud)
{
    PointBufferPtr point_buffer_ptr(new PointBuffer);
    lvr_ros::fromPointCloud2ToPointBuffer(*cloud, *point_buffer_ptr);

    lvr::ModelPtr pn(new lvr::Model);
    auto oldBuffer = boost::make_shared<lvr::PointBuffer>(
        point_buffer_ptr->toOldBuffer()
    );
    pn->m_pointCloud = oldBuffer;
    lvr::ModelFactory::saveModel(pn, "debug.ply");

    ROS_WARN("debug files saved!");
}

} /* namespace lvr_ros */


int main(int argc, char **args)
{
    ros::init(argc, args, "debug");
    lvr_ros::Debug debug;
    ros::spin();

    return 0;
}

