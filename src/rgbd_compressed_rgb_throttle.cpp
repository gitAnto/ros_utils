/*******************************************************************************

Software License Agreement (BSD 3-Clause)

Copyright (c) 2016, Antonio Coratelli, Antonio Petitti
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

#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/Image.h"

// General Defines
#define NAME "rgbd_compressed_rgb_throttle"

// Default Param Defines
#define RATE 5.0

// Input Defines
#define RGB_INFO_IN   "rgb/info_in"
#define RGB_COMPRESSED_IN   "rgb/compressed_in"
#define DEPTH_INFO_IN "depth/info_in"
#define DEPTH_IMAGE_IN "depth/image_in"
#define BUFFER_IN 1

// Output Defines
#define RGB_INFO_OUT   "rgb/info_out"
#define RGB_COMPRESSED_OUT   "rgb/compressed_out"
#define DEPTH_INFO_OUT "depth/info_out"
#define DEPTH_IMAGE_OUT "depth/image_out"
#define BUFFER_OUT 1

// Global Variables
ros::Publisher pub_rgb_info;
ros::Publisher pub_rgb_compressed;
ros::Publisher pub_depth_info;
ros::Publisher pub_depth_image;

double rate;
double secs;
double last_sent;

sensor_msgs::CameraInfo rgb_info;

sensor_msgs::CompressedImage rgb_compressed;
sensor_msgs::CompressedImage rgb_compressed_old;
sensor_msgs::CompressedImage rgb_compressed_oldd;

sensor_msgs::CameraInfo depth_info;

sensor_msgs::Image depth_image;
sensor_msgs::Image depth_image_old;
sensor_msgs::Image depth_image_oldd;

// Callbacks

void callback_rgb_info(const sensor_msgs::CameraInfo& data)
{
    rgb_info = data;
}

void callback_rgb_compressed(const sensor_msgs::CompressedImage& data)
{
    rgb_compressed_oldd = rgb_compressed_old;
    rgb_compressed_old = rgb_compressed;
    rgb_compressed = data;
}

void callback_depth_info(const sensor_msgs::CameraInfo& data)
{
    depth_info = data;
}

void callback_depth_image(const sensor_msgs::Image& data)
{
    depth_image_oldd = depth_image_old;
    depth_image_old = depth_image;
    depth_image = data;

    if (data.header.stamp.toSec() >= last_sent + secs)
    {
        ros::Time t_depth = depth_image.header.stamp;
        ros::Time t_rgb   = rgb_compressed.header.stamp;

        if(t_depth <= t_rgb)
        {
            sensor_msgs::CompressedImage rgb2pub;
            //t_depth as a ref
            if(t_depth == rgb_compressed.header.stamp)
                rgb2pub = rgb_compressed;
            else
                if(t_depth == rgb_compressed_old.header.stamp)
                    rgb2pub = rgb_compressed_old;
                else
                    if(t_depth == rgb_compressed_oldd.header.stamp)
                        rgb2pub = rgb_compressed_oldd;
                    else
                        ROS_WARN("Warning: no corrispondence!");

            pub_rgb_compressed.publish(rgb2pub);
            pub_depth_image.publish(depth_image);

        }
        else
        {
            //t_rgb as a ref
            sensor_msgs::Image depth2pub;
            if(t_rgb == depth_image.header.stamp)
                depth2pub = depth_image;
            else
                if(t_depth == depth_image_old.header.stamp)
                    depth2pub = depth_image_old;
                else
                    if(t_depth == depth_image_oldd.header.stamp)
                        depth2pub = depth_image_oldd;
                    else
                        ROS_WARN("Warning: no corrispondence!");

            pub_rgb_compressed.publish(rgb_compressed);
            pub_depth_image.publish(depth2pub);
        }

        pub_rgb_info.publish(rgb_info);
        pub_depth_info.publish(depth_info);
        last_sent = data.header.stamp.toSec();
    }
}

// Main

int main(int argc, char **argv)
{
    ros::init(argc, argv, NAME);
    ros::NodeHandle n("~");

    ROS_INFO("Initializing node '%s' ...", NAME);

    // Read Params
    n.param<double>("rate", rate, RATE);
    secs = 1.0 / rate;
    ROS_INFO("Time between frames: %f seconds.", secs);

    // Initialize Variables
    last_sent = 0;

    // Advertise Publishers
    pub_rgb_info   = n.advertise<sensor_msgs::CameraInfo>(RGB_INFO_OUT,   BUFFER_OUT);
    pub_rgb_compressed   = n.advertise<sensor_msgs::CompressedImage>(RGB_COMPRESSED_OUT,   BUFFER_OUT);
    pub_depth_info = n.advertise<sensor_msgs::CameraInfo>(DEPTH_INFO_OUT, BUFFER_OUT);
    pub_depth_image = n.advertise<sensor_msgs::Image>(DEPTH_IMAGE_OUT, BUFFER_OUT);

    ros::Subscriber sub_rgb_info   = n.subscribe(RGB_INFO_IN,   BUFFER_IN, callback_rgb_info);
    ros::Subscriber sub_rgb_compressed   = n.subscribe(RGB_COMPRESSED_IN,   BUFFER_IN, callback_rgb_compressed);
    ros::Subscriber sub_depth_info = n.subscribe(DEPTH_INFO_IN, BUFFER_IN, callback_depth_info);
    ros::Subscriber sub_depth_image = n.subscribe(DEPTH_IMAGE_IN, BUFFER_IN, callback_depth_image);

    ros::spin();
    return 0;
}

