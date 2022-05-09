#include <stdint.h>
#include <vector>
#include <chaos/common/data/Buffer.hpp>
namespace cv{
    class Mat;
}
namespace driver{
    namespace sensor{
         namespace camera{
             class Encoder:public chaos::common::data::Buffer{
                 public:
                std::vector<unsigned char> encbuf;

                 int sizex,sizey,offsetx,offsety;
                 
                 uint64_t ts;
                // uint64_t encts;
                 Encoder(){};
                 ~Encoder();
                 bool encode(const char*enc,cv::Mat&,std::vector<int>&);


             };
         }
         }
         }