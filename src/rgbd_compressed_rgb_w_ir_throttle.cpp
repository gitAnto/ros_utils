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
#define NAME "rgbd_compressed_rgb_w_ir_throttle"

// Default Param Defines
#define RATE 5.0

// Input Defines
#define RGB_INFO_IN   "rgb/info_in"
#define RGB_COMPRESSED_IN   "rgb/compressed_in"
#define DEPTH_INFO_IN "depth/info_in"
#define DEPTH_IMAGE_IN "depth/image_in"
#define IR_INFO_IN "ir/info_in"
#define IR_IMAGE_IN "ir/image_in"
#define IR2_INFO_IN "ir2/info_in"
#define IR2_IMAGE_IN "ir2/image_in"
#define BUFFER_IN 1

// Output Defines
#define RGB_INFO_OUT   "rgb/info_out"
#define RGB_COMPRESSED_OUT   "rgb/compressed_out"
#define DEPTH_INFO_OUT "depth/info_out"
#define DEPTH_IMAGE_OUT "depth/image_out"
#define IR_INFO_OUT "ir/info_out"
#define IR_IMAGE_OUT "ir/image_out"
#define IR2_INFO_OUT "ir2/info_out"
#define IR2_IMAGE_OUT "ir2/image_out"
#define BUFFER_OUT 1

// Global Variables
ros::Publisher pub_rgb_info;
ros::Publisher pub_rgb_compressed;
ros::Publisher pub_depth_info;
ros::Publisher pub_depth_image;
ros::Publisher pub_ir_info;
ros::Publisher pub_ir_image;
ros::Publisher pub_ir2_info;
ros::Publisher pub_ir2_image;

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

sensor_msgs::CameraInfo ir_info;

sensor_msgs::Image ir_image;
sensor_msgs::Image ir_image_old;
sensor_msgs::Image ir_image_oldd;

sensor_msgs::CameraInfo ir2_info;

sensor_msgs::Image ir2_image;
sensor_msgs::Image ir2_image_old;
sensor_msgs::Image ir2_image_oldd;

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

void callback_ir_info(const sensor_msgs::CameraInfo& data)
{
    ir_info = data;
}

void callback_ir_image(const sensor_msgs::Image& data)
{
    ir_image_oldd = ir_image_old;
    ir_image_old = ir_image;
    ir_image = data;
}

void callback_ir2_info(const sensor_msgs::CameraInfo& data)
{
    ir2_info = data;
}

void callback_ir2_image(const sensor_msgs::Image& data)
{
    ir2_image_oldd = ir2_image_old;
    ir2_image_old = ir2_image;
    ir2_image = data;
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
        //ros::Time t_depth = depth_image.header.stamp;
        //ros::Time t_rgb   = rgb_compressed.header.stamp;

        double stamps[4];
        stamps[0] = depth_image.header.stamp.toSec();
        stamps[1] = rgb_compressed.header.stamp.toSec();
        stamps[2] = ir_image.header.stamp.toSec();
        stamps[3] = ir2_image.header.stamp.toSec();

        int min_id = std::distance(stamps, std::min_element(stamps, stamps + 4));

        //ROS_INFO("min is in position: %d",min_id);

        switch(min_id)
        {
            case (0):
                {

                    //RGB
                    sensor_msgs::CompressedImage rgb2pub;
                    if(stamps[0] == rgb_compressed.header.stamp.toSec())
                        rgb2pub = rgb_compressed;
                    else
                        if(stamps[0] == rgb_compressed_old.header.stamp.toSec())
                            rgb2pub = rgb_compressed_old;
                        else
                            if(stamps[0] == rgb_compressed_oldd.header.stamp.toSec())
                                rgb2pub = rgb_compressed_oldd;
                            else
                                ROS_WARN("Warning: no corrispondence with rgb! -- case depth");
                    //IR
                    sensor_msgs::Image ir2pub;
                    if(stamps[0] == ir_image.header.stamp.toSec())
                        ir2pub = ir_image;
                    else
                        if(stamps[0] == ir_image_old.header.stamp.toSec())
                            ir2pub = ir_image_old;
                        else
                            if(stamps[0] == ir_image_oldd.header.stamp.toSec())
                                ir2pub = ir_image_oldd;
                            else
                                ROS_WARN("Warning: no corrispondence with ir! -- case depth");
                    //IR2
                    sensor_msgs::Image ir22pub;
                    if(stamps[0] == ir2_image.header.stamp.toSec())
                        ir22pub = ir2_image;
                    else
                        if(stamps[0] == ir2_image_old.header.stamp.toSec())
                            ir22pub = ir2_image_old;
                        else
                            if(stamps[0] == ir2_image_oldd.header.stamp.toSec())
                                ir22pub = ir2_image_oldd;
                            else
                                ROS_WARN("Warning: no corrispondence with ir2! -- case depth");


                    pub_rgb_compressed.publish(rgb2pub);
                    pub_ir_image.publish(ir2pub);
                    pub_ir2_image.publish(ir22pub);
                    pub_depth_image.publish(depth_image);
                    break;
                };

            case (1):
                {
                    //DEPTH
                    sensor_msgs::Image depth2pub;
                    if(stamps[1] == depth_image.header.stamp.toSec())
                        depth2pub = depth_image;
                    else
                        if(stamps[1] == depth_image_old.header.stamp.toSec())
                            depth2pub = depth_image_old;
                        else
                            if(stamps[1] == depth_image_oldd.header.stamp.toSec())
                                depth2pub = depth_image_oldd;
                            else
                                ROS_WARN("Warning: no corrispondence with depth! -- case rgb");
                    //IR
                    sensor_msgs::Image ir2pub;
                    if(stamps[1] == ir_image.header.stamp.toSec())
                        ir2pub = ir_image;
                    else
                        if(stamps[1] == ir_image_old.header.stamp.toSec())
                            ir2pub = ir_image_old;
                        else
                            if(stamps[1] == ir_image_oldd.header.stamp.toSec())
                                ir2pub = ir_image_oldd;
                            else
                                ROS_WARN("Warning: no corrispondence with ir! -- case rgb");
                    //IR2
                    sensor_msgs::Image ir22pub;
                    if(stamps[1] == ir2_image.header.stamp.toSec())
                        ir22pub = ir2_image;
                    else
                        if(stamps[1] == ir2_image_old.header.stamp.toSec())
                            ir22pub = ir2_image_old;
                        else
                            if(stamps[1] == ir2_image_oldd.header.stamp.toSec())
                                ir22pub = ir2_image_oldd;
                            else
                                ROS_WARN("Warning: no corrispondence with ir2! -- case rgb");

                    pub_rgb_compressed.publish(rgb_compressed);
                    pub_ir_image.publish(ir2pub);
                    pub_ir2_image.publish(ir22pub);
                    pub_depth_image.publish(depth2pub);
                    break;
                };

             
            case (2):
                {
                    //RGB
                    sensor_msgs::CompressedImage rgb2pub;
                    if(stamps[2] == rgb_compressed.header.stamp.toSec())
                        rgb2pub = rgb_compressed;
                    else
                        if(stamps[2] == rgb_compressed_old.header.stamp.toSec())
                            rgb2pub = rgb_compressed_old;
                        else
                            if(stamps[2] == rgb_compressed_oldd.header.stamp.toSec())
                                rgb2pub = rgb_compressed_oldd;
                            else
                                ROS_WARN("Warning: no corrispondence with rgb! -- case ir");

                    //DEPTH
                    sensor_msgs::Image depth2pub;
                    if(stamps[2] == depth_image.header.stamp.toSec())
                        depth2pub = depth_image;
                    else
                        if(stamps[2] == depth_image_old.header.stamp.toSec())
                            depth2pub = depth_image_old;
                        else
                            if(stamps[2] == depth_image_oldd.header.stamp.toSec())
                                depth2pub = depth_image_oldd;
                            else
                                ROS_WARN("Warning: no corrispondence with depth! -- case ir");

                    //IR2
                    sensor_msgs::Image ir22pub;
                    if(stamps[2] == ir2_image.header.stamp.toSec())
                        ir22pub = ir2_image;
                    else
                        if(stamps[2] == ir2_image_old.header.stamp.toSec())
                            ir22pub = ir2_image_old;
                        else
                            if(stamps[2] == ir2_image_oldd.header.stamp.toSec())
                                ir22pub = ir2_image_oldd;
                            else
                                ROS_WARN("Warning: no corrispondence with ir2! -- case ir");

                    pub_rgb_compressed.publish(rgb2pub);
                    pub_ir_image.publish(ir_image);
                    pub_ir2_image.publish(ir22pub);
                    pub_depth_image.publish(depth2pub);
                    break;
                };

         
            case (3):
                {
                    //RGB
                    sensor_msgs::CompressedImage rgb2pub;
                    if(stamps[3] == rgb_compressed.header.stamp.toSec())
                        rgb2pub = rgb_compressed;
                    else
                        if(stamps[3] == rgb_compressed_old.header.stamp.toSec())
                            rgb2pub = rgb_compressed_old;
                        else
                            if(stamps[3] == rgb_compressed_oldd.header.stamp.toSec())
                                rgb2pub = rgb_compressed_oldd;
                            else
                                ROS_WARN("Warning: no corrispondence with rgb! -- case ir2");

                    //DEPTH
                    sensor_msgs::Image depth2pub;
                    if(stamps[3] == depth_image.header.stamp.toSec())
                        depth2pub = depth_image;
                    else
                        if(stamps[3] == depth_image_old.header.stamp.toSec())
                            depth2pub = depth_image_old;
                        else
                            if(stamps[3] == depth_image_oldd.header.stamp.toSec())
                                depth2pub = depth_image_oldd;
                            else
                                ROS_WARN("Warning: no corrispondence with depth! -- case ir2");

                    //IR2
                    sensor_msgs::Image ir2pub;
                    if(stamps[3] == ir_image.header.stamp.toSec())
                        ir2pub = ir_image;
                    else
                        if(stamps[3] == ir_image_old.header.stamp.toSec())
                            ir2pub = ir_image_old;
                        else
                            if(stamps[3] == ir_image_oldd.header.stamp.toSec())
                                ir2pub = ir_image_oldd;
                            else
                                ROS_WARN("Warning: no corrispondence with ir! -- case ir2");

                    pub_rgb_compressed.publish(rgb2pub);
                    pub_ir_image.publish(ir2pub);
                    pub_ir2_image.publish(ir2_image);
                    pub_depth_image.publish(depth2pub);
                    break;
                };
             
            default:    
                ROS_WARN("Warning: something wrong in the sync callback!!");
        }

//	ROS_INFO("t_rgb: %f", t_rgb.toSec());
//	ROS_INFO("t_depth: %f", t_depth.toSec());
/*
        if(t_depth.toSec() <= t_rgb.toSec())
        {
            sensor_msgs::CompressedImage rgb2pub;
            //t_depth as a ref
            if(t_depth.toSec() == rgb_compressed.header.stamp.toSec())
                rgb2pub = rgb_compressed;
            else
                if(t_depth.toSec() == rgb_compressed_old.header.stamp.toSec())
                    rgb2pub = rgb_compressed_old;
                else
                    if(t_depth.toSec() == rgb_compressed_oldd.header.stamp.toSec())
                        rgb2pub = rgb_compressed_oldd;
                    else
                        ROS_WARN("Warning: no corrispondence! -- case depth");

            pub_rgb_compressed.publish(rgb2pub);
            pub_depth_image.publish(depth_image);

        }
        else
        {
            //t_rgb as a ref
            sensor_msgs::Image depth2pub;
            if(t_rgb.toSec() == depth_image.header.stamp.toSec())
                depth2pub = depth_image;
            else
                if(t_rgb.toSec() == depth_image_old.header.stamp.toSec())
                    depth2pub = depth_image_old;
                else
                    if(t_rgb.toSec() == depth_image_oldd.header.stamp.toSec())
                        depth2pub = depth_image_oldd;
                    else
                        ROS_WARN("Warning: no corrispondence! -- case rgb");

            pub_rgb_compressed.publish(rgb_compressed);
            pub_depth_image.publish(depth2pub);
        }
*/

        pub_rgb_info.publish(rgb_info);
        pub_depth_info.publish(depth_info);
        pub_ir_info.publish(ir_info);
        pub_ir2_info.publish(ir2_info);
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
    pub_ir_info = n.advertise<sensor_msgs::CameraInfo>(IR_INFO_OUT, BUFFER_OUT);
    pub_ir_image = n.advertise<sensor_msgs::Image>(IR_IMAGE_OUT, BUFFER_OUT);
    pub_ir2_info = n.advertise<sensor_msgs::CameraInfo>(IR2_INFO_OUT, BUFFER_OUT);
    pub_ir2_image = n.advertise<sensor_msgs::Image>(IR2_IMAGE_OUT, BUFFER_OUT);

    ros::Subscriber sub_rgb_info   = n.subscribe(RGB_INFO_IN,   BUFFER_IN, callback_rgb_info);
    ros::Subscriber sub_rgb_compressed   = n.subscribe(RGB_COMPRESSED_IN,   BUFFER_IN, callback_rgb_compressed);
    ros::Subscriber sub_depth_info = n.subscribe(DEPTH_INFO_IN, BUFFER_IN, callback_depth_info);
    ros::Subscriber sub_depth_image = n.subscribe(DEPTH_IMAGE_IN, BUFFER_IN, callback_depth_image);
    ros::Subscriber sub_ir_info = n.subscribe(IR_INFO_IN, BUFFER_IN, callback_ir_info);
    ros::Subscriber sub_ir_image = n.subscribe(IR_IMAGE_IN, BUFFER_IN, callback_ir_image);
    ros::Subscriber sub_ir2_info = n.subscribe(IR2_INFO_IN, BUFFER_IN, callback_ir2_info);
    ros::Subscriber sub_ir2_image = n.subscribe(IR2_IMAGE_IN, BUFFER_IN, callback_ir2_image);

    ros::spin();
    return 0;
}

