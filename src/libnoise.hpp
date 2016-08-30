#ifndef LIBNOISE_H_
#define LIBNOISE_H_

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


    class AbstractNoise
    {
    public:
        virtual Pose run (Pose p) = 0;
        virtual bool setParam(std::string key, double value) = 0;
    };


    class GaussianNoise : public AbstractNoise
    {
    private:

        double mean;
        double stddev;

        boost::mt19937 rng;
        boost::normal_distribution<> *normal;
        boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > *generator;

        void reset()
        {
            this->normal = new boost::normal_distribution(this->mean, this->stddev);
            this->generator = new boost::variate_generator(this->rng, *(this->normal));
        }

    public:

        GaussianNoise(double mean = 0.0, double stddev = 1.0)
        {
            this->mean = mean;
            this->stddev = stddev;
            this->reset();
        }

        bool setParam(std::string key, double value)
        {
            if (key == 'mean') { this->mean = value; this->reset(); return true; }
            if (key == 'stddev') { this->stddev = value; this->reset(); return true; }
            return false;
        }

        Pose run(Pose in)
        {
            Pose out;

            out.x = in.x + (*generator)();
            out.y = in.y + (*generator)();
            out.z = in.z + (*generator)();

            return out;
        }

    };


    class NoiseGenerator
    {
    private:
        AbstractNoise* pNoise;

    public:

        NoiseGenerator (AbstractNoise& noise_type)
        {
            this->setNoise(noise_type);
        }

        void setNoise(AbstractNoise& noise_type)
        {
            this->pNoise = &noise_type;
        }

        Pose generateNoise(Pose in)
        {
            return this->pNoise->run(in);
        }
    };
}

#endif
