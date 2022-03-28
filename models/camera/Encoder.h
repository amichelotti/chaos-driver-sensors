#include <stdint.h>
#include <vector>

namespace cv{
    class Mat;
}
namespace driver{
    namespace sensor{
         namespace camera{
             struct Encoder{
                std::vector<unsigned char> encbuf;

                 int sizex,sizey,offsetx,offsety;
                 void*ptr;
                 size_t size;
                 uint64_t ts;
                 Encoder():ptr(NULL),size(0){};
                 ~Encoder();
                 bool encode(const char*enc,cv::Mat&,std::vector<int>&);


             };
         }
         }
         }