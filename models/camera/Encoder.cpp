#include "Encoder.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <chaos/common/utility/TimingUtil.h>
namespace driver{
    namespace sensor{
         namespace camera{
#pragma GCC diagnostic push
#pragma GCC diagnostic warning "-fpermissive"
#include "spng.c"
#pragma GCC diagnostic pop

bool Encoder::encode(const char *encoding, cv::Mat &ma, std::vector<int> &encode_params) {
  {
    bool ret;
    sizex=ma.cols;
    sizey=ma.rows;

    if (!strcmp(encoding, ".png")) {
      spng_ctx        *ctx;
      int              ret;
      struct spng_ihdr ihdr = {0};
      ctx                   = spng_ctx_new(SPNG_CTX_ENCODER);
      spng_set_option(ctx, SPNG_ENCODE_TO_BUFFER, 1);
      ihdr.width     = ma.cols;
      ihdr.height    = ma.rows;
      ihdr.bit_depth = 8;

      if (ma.channels() == 1) {
        ihdr.color_type = SPNG_COLOR_TYPE_GRAYSCALE;
        switch (ma.depth()) {
          case CV_16U:
          case CV_16S:
            ihdr.bit_depth = 16;
            break;
          default:
            ihdr.bit_depth = 8;
        }
      } else {
        ihdr.color_type = SPNG_COLOR_TYPE_TRUECOLOR;
      }
      /* Valid color type, bit depth combinations: https://www.w3.org/TR/2003/REC-PNG-20031110/#table111 */

      spng_set_ihdr(ctx, &ihdr);

      /* When encoding fmt is the source format */
      /* SPNG_FMT_PNG is a special value that matches the format in ihdr */

      /* SPNG_ENCODE_FINALIZE will finalize the PNG with the end-of-file marker */
      ret = spng_encode_image(ctx, ma.ptr(), ma.total() * ma.elemSize(), SPNG_FMT_PNG, SPNG_ENCODE_FINALIZE);
      if (ret) {
        printf("spng_encode_image() error: %s\n", spng_strerror(ret));
        spng_ctx_free(ctx);

        return false;
      }
      ptr = spng_get_png_buffer(ctx, &size, &ret);
      ts=chaos::common::utility::TimingUtil::getTimeStampInMicroseconds();
      spng_ctx_free(ctx);

      return (ptr!=NULL);
    } else {
      ret = cv::imencode(encoding, ma, encbuf, encode_params);
      if (ret) {
        ptr  = reinterpret_cast<uchar *>(&(encbuf[0]));
        size = encbuf.size();
      }
      ts=chaos::common::utility::TimingUtil::getTimeStampInMicroseconds();

      return ret;
    }
  }
}
 Encoder::~Encoder(){
     if(ptr&&(encbuf.size()==0)){
        free(ptr);

     }
 }        
         }}}