/*******************************************************************************
 *
 *  Copyright (c) 2016, Antonio Coratelli
 *
 *  Software License Agreement (BSD License)
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 * 		 the distribution.
 *   * Neither the name of the athor nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Author: Antonio Coratelli <antoniocoratelli@gmail.com>
 *  Last edit: May, 2016
 ******************************************************************************/

#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"


// General Defines

#define NAME "rgbd_throttle"


// Default Param Defines

#define RATE 5.0


// Input Defines

#define RGB_INFO_IN   "rgb/info_in"
#define RGB_RECT_IN   "rgb/rect_in"
#define DEPTH_INFO_IN "depth/info_in"
#define DEPTH_RECT_IN "depth/rect_in"

#define BUFFER_IN 1


// Output Defines

#define RGB_INFO_OUT   "rgb/info_out"
#define RGB_RECT_OUT   "rgb/rect_out"
#define DEPTH_INFO_OUT "depth/info_out"
#define DEPTH_RECT_OUT "depth/rect_out"

#define BUFFER_OUT 1


// Global Variables

ros::Publisher  pub_rgb_info;
ros::Publisher  pub_rgb_rect;
ros::Publisher  pub_depth_info;
ros::Publisher  pub_depth_rect;

double rate;
double secs;

double last_rgb_info;

bool accept_rgb_info;
bool accept_rgb_rect;
bool accept_depth_info;
bool accept_depth_rect;

void callback_rgb_info(const sensor_msgs::CameraInfo& data)
{
	if (data.header.stamp.toSec() <  last_rgb_info - secs)
	{
		last_rgb_info = 0;
	}
	
	if (data.header.stamp.toSec() >= last_rgb_info + secs)
	{
		accept_rgb_info   = true;
		accept_rgb_rect   = false;
		accept_depth_info = true; // TODO is it possible to enable true?
		accept_depth_rect = false;
	}
	
	if (accept_rgb_info == true)
	{
		pub_rgb_info.publish(data);
		last_rgb_info     = data.header.stamp.toSec();
		accept_rgb_info   = false;
		accept_rgb_rect   = true;
	}
}

void callback_rgb_rect(const sensor_msgs::Image& data)
{
	if (accept_rgb_rect == true)
	{
		pub_rgb_rect.publish(data);
		accept_rgb_rect = false;
	}
}

void callback_depth_info(const sensor_msgs::CameraInfo& data)
{
	if (accept_depth_info == true)
	{
		pub_depth_info.publish(data);
		accept_depth_info = false;
		accept_depth_rect = true;
	}
}

void callback_depth_rect(const sensor_msgs::Image& data)
{
	if (accept_depth_rect == true)
	{
		pub_depth_rect.publish(data);
		accept_depth_rect = false;
	}
}


// Main

int main(int argc, char **argv)
{
	ros::init(argc, argv, NAME);
	ros::NodeHandle n;
	
	ROS_INFO("Initializing node '%s' ...", NAME);
	
	// Read Params
	n.param<double>("rate", rate, RATE); secs = 1.0 / rate;
	ROS_INFO("Time between frames: %f seconds.", secs);
	
	// Initialize Variables
	bool accept_rgb_info   = true;
	bool accept_rgb_rect   = false;
	bool accept_depth_info = false;
	bool accept_depth_rect = false;
	
	// Advertise Publishers
	pub_rgb_info   = n.advertise<sensor_msgs::CameraInfo>(RGB_INFO_OUT,   BUFFER_OUT);
	pub_rgb_rect   = n.advertise<sensor_msgs::Image     >(RGB_RECT_OUT,   BUFFER_OUT);
	pub_depth_info = n.advertise<sensor_msgs::CameraInfo>(DEPTH_INFO_OUT, BUFFER_OUT);
	pub_depth_rect = n.advertise<sensor_msgs::Image     >(DEPTH_RECT_OUT, BUFFER_OUT);
	
	ros::Subscriber sub_rgb_info   = n.subscribe(RGB_INFO_IN,   BUFFER_IN, callback_rgb_info);
	ros::Subscriber sub_rgb_rect   = n.subscribe(RGB_RECT_IN,   BUFFER_IN, callback_rgb_rect);
	ros::Subscriber sub_depth_info = n.subscribe(DEPTH_INFO_IN, BUFFER_IN, callback_depth_info);
	ros::Subscriber sub_depth_rect = n.subscribe(DEPTH_RECT_IN, BUFFER_IN, callback_depth_rect);
	
	ros::spin();
	return 0;
}

