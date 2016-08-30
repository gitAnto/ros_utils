#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>

namespace libnoise{ 

  struct Pose {
    double x;
    double y;
    double z;

    double qx;
    double qy;
    double qz;
    double qw;
  };


  class AbstractNoise { // abstract class
  public:
    virtual Pose run (Pose p) = 0;
    virtual bool setParam(std::string key, double value); 
  };
   
  class GaussianNoise:public AbstractNoise{
  private:
    double mean = 0;
    double std_dev = 1;  
    
    boost::mt19937 rng; // I don't seed it on purpouse (it's not relevant)

    boost::normal_distribution<> *nd;

    boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > *var_nor(rng, *nd);


  public:

    GaussianNoise(){
       nd = new boost::normal_distribution<> (mean,std_dev);   
       var_nor = new boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > (rng, *nd);
    }
    
    bool setParam(std::string key, double value){

      if (key == 'mean'){
        mean = value;
        nd = new boost::normal_distribution<> (mean,std_dev);
				var_nor = new boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > (rng, *nd);
        return true;     
      }
      if (key == 'std_dev'){
        std_dev = value;
        nd = new boost::normal_distribution<> (mean,std_dev);
				var_nor = new boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > (rng, *nd);
        return true;   
      }
      return false;
    }    
 
    
    Pose run(Pose p){

      Pose p_noise;

      p_noise.x = p.x + (*var_nor)();
      p_noise.y = p.y + (*var_nor)();
      p_noise.z = p.z + (*var_nor)();
      
      return p_noise;
    }

  };


	class NoiseGenerator{
	private:
	  AbstractNoise* pNoise;
	 
	public:
	  NoiseGenerator (AbstractNoise& noise_type) : pNoise(&noise_type){}

	  void SetNoise(AbstractNoise& noise_type){
	    pNoise = &noise_type;
	  }
	 
	  Pose generateNoise(Pose p){
	    return pNoise->run(p);
	  }
};
}
