#ifndef LIBNOISE_CONVERSIONS_H_
#define LIBNOISE_CONVERSIONS_H_

/*******************************************************************************

Software License Agreement (BSD 3-Clause)

Copyright (c) 2016, Antonio Coratelli, Donato Di Paola
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors
   may be used to endorse or promote products derived from this software
   without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

*******************************************************************************/

#include "libnoise.hpp"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TransformStamped.h"

namespace libnoise
{
    template <typename T> Pose fromROS(T) { /**/ }

    template <typename T> T toROS(Pose, T) { /**/ }

    /* geometry_msgs::Pose ****************************************************/

    template <> Pose fromROS(const geometry_msgs::Pose in)
    {
        Pose out;
        out.x = in.position.x;
        out.y = in.position.y;
        out.z = in.position.z;
        out.qx = in.orientation.x;
        out.qy = in.orientation.y;
        out.qz = in.orientation.z;
        out.qw = in.orientation.w;
        return out;
    }

    template <> geometry_msgs::Pose toROS(Pose in, geometry_msgs::Pose base)
    {
        base.position.x = in.x;
        base.position.y = in.y;
        base.position.z = in.z;
        base.orientation.x = in.qx;
        base.orientation.y = in.qy;
        base.orientation.z = in.qz;
        base.orientation.w = in.qw;
        return base;
    }

    /* geometry_msgs::PoseStamped *********************************************/

    template <> Pose fromROS(const geometry_msgs::PoseStamped in)
    {
        return fromROS(in.pose);
    }

    template <> geometry_msgs::PoseStamped toROS(Pose in, geometry_msgs::PoseStamped base)
    {
        base.pose = toROS(in, base.pose);
        return base;
    }

    /* geometry_msgs::Transform ***********************************************/

    template <> Pose fromROS(const geometry_msgs::Transform in)
    {
        Pose out;
        out.x = in.translation.x;
        out.y = in.translation.y;
        out.z = in.translation.z;
        out.qx = in.rotation.x;
        out.qy = in.rotation.y;
        out.qz = in.rotation.z;
        out.qw = in.rotation.w;
        return out;
    }

    template <> geometry_msgs::Transform toROS(Pose in, geometry_msgs::Transform base)
    {
        base.translation.x = in.x;
        base.translation.y = in.y;
        base.translation.z = in.z;
        base.rotation.x = in.qx;
        base.rotation.y = in.qy;
        base.rotation.z = in.qz;
        base.rotation.w = in.qw;
        return base;
    }

    /* geometry_msgs::TransformStamped ****************************************/

    template <> Pose fromROS(const geometry_msgs::TransformStamped in)
    {
        return fromROS(in.transform);
    }

    template <> geometry_msgs::TransformStamped toROS(Pose in, geometry_msgs::TransformStamped base)
    {
        base.transform = toROS(in, base.transform);
        return base;
    }

}
#endif
