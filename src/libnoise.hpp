#ifndef LIBNOISE_H_
#define LIBNOISE_H_

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

#include <ctime>
#include <cmath>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>

namespace libnoise
{
    struct Pose
    {
        double x;
        double y;
        double z;
        double qx;
        double qy;
        double qz;
        double qw;
    };


    template <typename A, typename B> Pose random_quaternion(boost::variate_generator<A,B> generator)
    {
        Pose out;
        out.x = 0.0;
        out.y = 0.0;
        out.z = 0.0;
        out.qx = generator();
        out.qy = generator();
        out.qz = generator();
        out.qw = generator();
        double module = std::sqrt(out.qx*out.qx + out.qy*out.qy + out.qz*out.qz + out.qw*out.qw);
        out.qx /= module;
        out.qy /= module;
        out.qz /= module;
        out.qw /= module;
        return out;
    }


    Pose quaternion_slerp(Pose a, Pose b, double t)
    {
        Pose r;
        r.x = 0.0;
        r.y = 0.0;
        r.z = 0.0;
        if (t > 1.0) t = 1.0;
        if (t < 0.0) t = 0.0;
    	double t_ = 1 - t;
    	double Wa, Wb;
    	double theta = acos(a.qx*b.qx + a.qy*b.qy + a.qz*b.qz + a.qw*b.qw);
    	double sn = sin(theta);
    	Wa = std::sin(t_*theta) / sn;
    	Wb = std::sin(t*theta) / sn;
    	r.qx = Wa*a.qx + Wb*b.qx;
    	r.qy = Wa*a.qy + Wb*b.qy;
    	r.qz = Wa*a.qz + Wb*b.qz;
    	r.qw = Wa*a.qw + Wb*b.qw;
    	double module = std::sqrt(r.qx*r.qx + r.qy*r.qy + r.qz*r.qz + r.qw*r.qw);
        r.qx /= module;
        r.qy /= module;
        r.qz /= module;
        r.qw /= module;
    	return r;
    }


    class AbstractNoise
    {
    public:
        virtual Pose run(Pose) = 0;
    };


    class GaussianNoise : public AbstractNoise
    {
    protected:
        boost::mt19937 rng;
        boost::normal_distribution<> normal;
        boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > generator;


    public:

        GaussianNoise(double mean = 0.0, double stddev = 1.0)
        : normal(mean, stddev)
        , generator(this->rng, this->normal)
        , rng(std::time(0))
        {
        }

        Pose run(Pose in)
        {
            Pose out;
            Pose quat = random_quaternion(this->generator);
            out = quaternion_slerp(in, quat, generator());
            out.x = in.x + generator();
            out.y = in.y + generator();
            out.z = in.z + generator();
            return out;
        }

    };


    class NoiseGenerator
    {
    private:
        AbstractNoise* pNoise = NULL;

    public:

        NoiseGenerator()
        {
        }

        NoiseGenerator(AbstractNoise& noise_type)
        {
            this->setNoiseType(noise_type);
        }

        void setNoiseType(AbstractNoise& noise_type)
        {
            this->pNoise = &noise_type;
        }

        void setNoiseType(AbstractNoise* noise_type)
        {
            this->pNoise = noise_type;
        }

        Pose addNoise(Pose in)
        {
            return this->pNoise->run(in);
        }
    };
}

#endif
