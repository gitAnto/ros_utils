/*******************************************************************************

Software License Agreement (BSD 3-Clause)

Copyright (c) 2016, Antonio Coratelli
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
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "libnoise.hpp"
#include "libnoise_conversions.hpp"

// Constants
#define NAME "pose_noise_generator"
#define BUFFER_IN 1
#define BUFFER_OUT 1


libnoise::NoiseGenerator noise_generator;

ros::Publisher publisher;
ros::Subscriber subscriber;


// Callbacks

template <typename T> void callback(const T in)
{
    T out = in;
    libnoise::Pose p_in = libnoise::fromROS<T>(in);
    libnoise::Pose p_out = noise_generator.addNoise(p_in);
    out = libnoise::toROS<T>(p_out, out);
    publisher.publish(out);
}


// Main

int main(int argc, char **argv)
{
    ros::init(argc, argv, NAME);
    ros::NodeHandle nh;
    ros::NodeHandle np("~");

    ROS_INFO("Initializing node '%s' ...", NAME);


    std::string noise_type = "Gaussian";
    np.param<std::string>("noise_type", noise_type, noise_type);

    if (noise_type == "Gaussian")
    {
        double noise_mean = 0.0;
        double noise_stddev = 1.0;

        np.param<double>("noise_mean", noise_mean, noise_mean);
        np.param<double>("noise_stddev", noise_stddev, noise_stddev);

        noise_generator.setNoiseType(new libnoise::GaussianNoise(noise_mean, noise_stddev));

        ROS_INFO("- noise_type: %s", noise_type.c_str());
    }
    else { ROS_FATAL("Unkown noise_type '%s'", noise_type.c_str()); return 1; }


    std::string message_type = "geometry_msgs/Pose";
    np.param<std::string>("message_type", message_type, message_type);

    if (message_type == "geometry_msgs/Pose")
    {
        publisher = nh.advertise<geometry_msgs::Pose>("out", BUFFER_OUT);
        subscriber = nh.subscribe("in", BUFFER_IN, callback<geometry_msgs::Pose>);

        ROS_INFO("- message_type: %s", message_type.c_str());
    }
    else if (message_type == "geometry_msgs/PoseStamped")
    {
        publisher = nh.advertise<geometry_msgs::PoseStamped>("out", BUFFER_OUT);
        subscriber = nh.subscribe("in", BUFFER_IN, callback<geometry_msgs::PoseStamped>);
    }
    else { ROS_FATAL("Unkown message_type '%s'", message_type.c_str()); return 1; }

    ros::spin();
    return 0;
}
