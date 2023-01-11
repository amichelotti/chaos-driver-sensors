/*
 *	AbstracCameraDriver.cpp
 *	!CHAOS
 *	Created by Andrea Michelotti
 *  Abstraction of a simple sensor driver
 *    	Copyright 2018 INFN, National Institute of Nuclear Physics
 *
 *    	Licensed under the Apache License, Version 2.0 (the "License");
 *    	you may not use this file except in compliance with the License.
 *    	You may obtain a copy of the License at
 *
 *    	http://www.apache.org/licenses/LICENSE-2.0
 *
 *    	Unless required by applicable law or agreed to in writing, software
 *    	distributed under the License is distributed on an "AS IS" BASIS,
 *    	WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *    	See the License for the specific language governing permissions and
 *    	limitations under the License.
 */
#include "AbstractCameraDriver.h"
#ifdef CAMERA
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#define RCVENCODING(e, cvenc) \
   switch(cvenc){case CV_8UC4:case CV_8SC4:bpp=4;break;\
    case CV_8UC3:case CV_8SC3:bpp=3;break;\
    case CV_8UC2:case CV_8SC2:bpp=2;break;\
    case CV_8UC1:case CV_8SC1:bpp=1;break;\
    case CV_16UC1:case CV_16SC1:bpp=2;break;\
    case CV_16UC2:case CV_16SC2:bpp=4;break;\
    case CV_16UC3:case CV_16SC3:bpp=6;break;\
    case CV_32SC1:bpp=4;break;\
    case CV_32SC2:bpp=8;break;\
    case CV_32SC3:bpp=16;break;\
    default:bpp=16;}}
#define CVENCODING(e, cvenc)                                            \
  if (e == #cvenc) { return   cvenc;}


   

#else
#define RCVENCODING(e, cvenc)
#define DECODE_CVENCODING(e, cvenc) 
#define CVENCODING(e, cvenc)                                            
#define CV_8UC1 0


#endif
namespace driver {
namespace sensor {
namespace camera{
#define AbstractCameraDriverLAPP_		LAPP_ << "[AbstractCameraDriver] "
#define AbstractCameraDriverLDBG_		LDBG_ << "[AbstractCameraDriver:"<<__FUNCTION__<<"]"
#define AbstractCameraDriverLERR_		LERR_ << "[AbstractCameraDriver:"<<__PRETTY_FUNCTION__<<"]"



int fmt2cv(const std::string& enc){
    
      CVENCODING(enc, CV_8UC4);
      CVENCODING(enc, CV_8UC3);
      CVENCODING(enc, CV_8UC2);
      CVENCODING(enc, CV_8UC1);
      CVENCODING(enc, CV_8SC1);
      CVENCODING(enc, CV_8SC2);
      CVENCODING(enc, CV_8SC3);
      CVENCODING(enc, CV_8SC4);

      CVENCODING(enc, CV_16UC4);
      CVENCODING(enc, CV_16UC3);
      CVENCODING(enc, CV_16UC2);
      CVENCODING(enc, CV_16UC1);
      CVENCODING(enc, CV_16SC1);
      CVENCODING(enc, CV_16SC2);
      CVENCODING(enc, CV_16SC3);
      CVENCODING(enc, CV_16SC4);
      CVENCODING(enc, CV_32SC1);
      CVENCODING(enc, CV_32SC2);
      CVENCODING(enc, CV_32SC3);
      CVENCODING(enc, CV_32SC4);
      if(enc=="BAYERBG8"){
          return cv::COLOR_BayerBG2RGB ;
      } else if(enc== "BayerBG16"){
          return cv::COLOR_BayerBG2RGB|0x1000 ;

      } 
      if(enc=="YUV422packed"){
          return cv::COLOR_YUV2RGB_NV21;
      }
      return CV_8UC1;
}
int cv2fmt(int cvenc, std::string& enc){
    int bpp=1;
    switch(cvenc){
        case cv::COLOR_YUV2RGB_NV21:
            bpp=2;enc="YUV422packed";
            break;
        case cv::COLOR_BayerBG2RGB:
        bpp=1;enc="BAYERBG8";
        break;
        case (0x1000|cv::COLOR_BayerBG2RGB):
        bpp=2;enc="BAYERBG16";
        break;
        case CV_8UC4:
            bpp=4;enc="CV_8UC4";
            break;
        case CV_8SC4:
            bpp=4;enc="CV_8SC4";
        break;
    case CV_8UC3:
        bpp=3;
        enc="CV_8UC3";
    break;
    case CV_8SC3:
        bpp=3;
        enc="CV_8SC3";

    break;
    case CV_8UC2:
        bpp=2;enc="CV_8UC2";
        break;
    case CV_8SC2:
        bpp=2;enc="CV_8USC2";
        break;
    case CV_8UC1:
        bpp=1;enc="CV_8UC1";
        break;
    case CV_8SC1:
        bpp=1;enc="CV_8SC1";
        break;
    case CV_16UC1:
        bpp=2;enc="CV_16UC1";
        break;
    case CV_16SC1:
        bpp=2;enc="CV_16SC1";
        break;
    case CV_16UC2:
        bpp=4;        
        enc="CV_16UC2";
        break;
    case CV_16SC2:
        bpp=4;
        enc="CV_16SC2";

        break;
    case CV_16UC3:
        bpp=6;
        enc="CV_16UC3";

        break;
    case CV_16SC3:
        bpp=6;
        enc="CV_16SC3";

        break;
    case CV_32SC1:
        bpp=4;
        enc="CV_32SC1";

        break;
    case CV_32SC2:
        bpp=8;        
        enc="CV_32SC2";

        break;
    case CV_32SC3:
        bpp=16;
        enc="CV_32SC3";

        break;
    default:
        bpp=16;
        enc="UKNOWN";

        }
    return bpp;
}
void AbstractCameraDriver::parseInitCommonParams(const chaos::common::data::CDataWrapper& config){
    AbstractCameraDriverLDBG_<<"config:"<<config.getCompliantJSONString();
    if(config.hasKey(TRIGGER_HW_SOURCE_KEY)){
        triggerHWSource=config.getStringValue(TRIGGER_HW_SOURCE_KEY);
    }
    if(config.hasKey(SERIAL_KEY)){
        serial=config.getStringValue(SERIAL_KEY);
        AbstractCameraDriverLDBG_<<"seria:"<<serial;

    }
    if(config.hasKey(TRIGGER_HW_TIMEOUT_KEY)){
        hw_trigger_timeout_us=config.getInt32Value(TRIGGER_HW_TIMEOUT_KEY);
    }
    if(config.hasKey(TRIGGER_SW_TIMEOUT_KEY)){
        sw_trigger_timeout_us=config.getInt32Value(TRIGGER_SW_TIMEOUT_KEY);
    }
     if(config.hasKey(TRIGGER_MODE_KEY)){
        tmode=(TriggerModes)config.getInt32Value(TRIGGER_MODE_KEY);
    }
    if(config.hasKey(GRAB_STRATEGY_KEY)){
        gstrategy=(GrabStrategy)config.getInt32Value("GRAB_STRATEGY");
    }

   
}

}
}
}
