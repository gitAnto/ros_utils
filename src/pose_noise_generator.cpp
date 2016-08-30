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

// General Defines
#define NAME "pose_noise_generator"

libnoise::NoiseGenerator noise_generator;

ros::Publisher publisher;
ros::Subscriber subscriber;


// Callbacks

template <typename T> void callback(T in)
{
    libnoise::Pose p_in = libnoise::import<T>(in);
    libnoise::Pose p_out = noise_generator.addNoise(p)
    publisher.publish(libnoise::export<T>(p_out, in));
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
    }
    else throw new Exception("Unkown noise_type");


    std::string message_type = "geometry_msgs/Pose";
    np.param<std::string>("message_type", message_type, message_type);

    if (message_type == "geometry_msgs/Pose")
    {
        publisher = nh.advertise<sensor_msgs::CameraInfo>("out", BUFFER_OUT);
        subscriber = nh.subscribe("in", BUFFER_IN, callback);
    }
    else throw new Exception("Unkown message_type");

    ros::spin();
    return 0;
}
