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
 * debug.h
 *
 * Author: Johan M. von Behren <johan@vonbehren.eu>
 *
 */

#ifndef LVR_ROS_RECONSTRUCTION_H_
#define LVR_ROS_RECONSTRUCTION_H_

#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>
#include <lvr2/geometry/BaseVector.hpp>
#include <lvr2/io/PointBuffer.hpp>

namespace lvr_ros
{

using Vec = lvr2::BaseVector<float>;
using PointBuffer = lvr2::PointBuffer<Vec>;
using PointBufferPtr = lvr2::PointBufferPtr<Vec>;

class Debug
{
public:
    Debug();
    ~Debug();

private:
    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr cloud);
    void dumpPointCloudToFile(const sensor_msgs::PointCloud2::ConstPtr cloud);

    ros::NodeHandle node_handle;
    ros::Subscriber cloud_subscriber;
};

} /* namespace lvr_ros */

#endif /* LVR_ROS_RECONSTRUCTION_H_ */
