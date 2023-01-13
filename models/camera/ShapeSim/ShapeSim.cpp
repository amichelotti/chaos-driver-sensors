/*
 *	ShapeSim.h
 *	!CHAOS
 *	Created by Andrea Michelotti.
 *
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
#include "ShapeSim.h"
#include <chaos/common/data/CDataWrapper.h>
#include <chaos/cu_toolkit/driver_manager/driver/AbstractDriverPlugin.h>
#include <math.h>
#include <stdlib.h>
#include <boost/lexical_cast.hpp>
#include <string>

#include <chaos/common/data/structured/DatasetAttribute.h>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int_distribution.hpp>
#include <boost/random/uniform_real.hpp>

#include "../beamFunc.h"

namespace cu_driver = chaos::cu::driver_manager::driver;
using namespace ::driver::sensor::camera;
using namespace chaos::common::data::structured;

using namespace cv;

#define ShapeSimLAPP_ LAPP_ << "[ShapeSim] "
#define ShapeSimLDBG_ LDBG_ << "[ShapeSim:" << __PRETTY_FUNCTION__ << "]"
#define ShapeSimLERR_ LERR_ << "[ShapeSim:" << __PRETTY_FUNCTION__ << "]"

#define CREATE_INT_PROP(n, pub, var, min, max, inc) \
  createProperty(                                   \
      n, var, (int32_t)min, (int32_t)max, (int32_t)inc, pub, [](AbstractDriver* thi, const std::string& name, const chaos::common::data::CDataWrapper& p) -> chaos::common::data::CDWUniquePtr {\
        chaos::common::data::CDWUniquePtr ret(new chaos::common::data::CDataWrapper());\
        ret->addInt32Value(PROPERTY_VALUE_KEY,((ShapeSim*)thi)->var);\
        return ret; }, [](AbstractDriver* thi, const std::string& name, const chaos::common::data::CDataWrapper& p) -> chaos::common::data::CDWUniquePtr { \
          ((ShapeSim*)thi)->var=p.getInt32Value(PROPERTY_VALUE_KEY);\
          return p.clone(); });

#define CREATE_DOUBLE_PROP(n, pub, var, min, max, inc) \
  createProperty(                                      \
      n, var, min, max, inc, pub, [](AbstractDriver* thi, const std::string& name, const chaos::common::data::CDataWrapper& p) -> chaos::common::data::CDWUniquePtr {\
        chaos::common::data::CDWUniquePtr ret(new chaos::common::data::CDataWrapper());\
        ret->addDoubleValue(PROPERTY_VALUE_KEY,((ShapeSim*)thi)->var);\
        return ret; }, [](AbstractDriver* thi, const std::string& name, const chaos::common::data::CDataWrapper& p) -> chaos::common::data::CDWUniquePtr { \
          ((ShapeSim*)thi)->var=p.getDoubleValue(PROPERTY_VALUE_KEY);\
          return p.clone(); });

#define GETDBLPARAM(w, name)                                        \
  if (w->hasKey(#name)) {                                           \
    name = w->getDoubleValue(#name);                                \
    ShapeSimLDBG_ << "shape DBL param '" << #name << " = " << name; \
  }
#define GETINTPARAM(w, name)                                        \
  if (w->hasKey(#name)) {                                           \
    name = w->getInt32Value(#name);                                 \
    ShapeSimLDBG_ << "shape INT param '" << #name << " = " << name; \
  }

// GET_PLUGIN_CLASS_DEFINITION
// we need only to define the driver because we don't are makeing a plugin
OPEN_CU_DRIVER_PLUGIN_CLASS_DEFINITION(ShapeSim, 1.0.0, ::driver::sensor::camera::ShapeSim)
REGISTER_CU_DRIVER_PLUGIN_CLASS_INIT_ATTRIBUTE(::driver::sensor::camera::ShapeSim, http_address / dnsname
                                               : port)
CLOSE_CU_DRIVER_PLUGIN_CLASS_DEFINITION
OPEN_REGISTER_PLUGIN
REGISTER_PLUGIN(::driver::sensor::camera::ShapeSim)
CLOSE_REGISTER_PLUGIN

void ShapeSim::driverInit(const char* initParameter) throw(chaos::CException) {
  throw chaos::CException(-1, "cannot intialize camera expected bad JSON parameters" + std::string(initParameter), __PRETTY_FUNCTION__);
}

void ShapeSim::driverInit(const chaos::common::data::CDataWrapper& json) throw(chaos::CException) {
  ShapeSimLDBG_ << "Initializing  driver:" << json.getCompliantJSONString();
  ShapeSimLDBG_ << "Inital properties" << getProperties()->getJSONString();
  if (initializeCamera(json) != 0) {
    throw chaos::CException(-1, "cannot initialize camera " + json.getCompliantJSONString(), __PRETTY_FUNCTION__);
  }
}

void ShapeSim::driverDeinit() throw(chaos::CException) {
  ShapeSimLAPP_ << "Deinit driver";
}

namespace driver {

namespace sensor {
namespace camera {

// GET_PLUGIN_CLASS_DEFINITION
// we need to define the driver with alias version and a class that implement it
int ShapeSim::initializeCamera(const chaos::common::data::CDataWrapper& json) {
  int ret = -2;
  ShapeSimLDBG_ << "intialize camera:" << json.getCompliantJSONString();
  serial_id = rand();
  propsToCamera((chaos::common::data::CDataWrapper*)&json);

  
  if (json.hasKey("SHAPE")) {
    if (json.isCDataWrapperValue("SHAPE")) {
      shape_params = json.getCSDataValue("SHAPE");
      if (shape_params != NULL) {
        if (shape_params->hasKey("width")) {
          width = shape_params->getInt32Value("width");
        }
        if (shape_params->hasKey("height")) {
          height = shape_params->getInt32Value("height");
        }
        if (shape_params->hasKey("type")) {
          ret         = 0;
          initialized = true;
        } else {
          ShapeSimLERR_ << "Missing shape 'type'" << json.getCompliantJSONString();
        }

      } else {
        ShapeSimLERR_ << "  retriving 'SHAPE'" << json.getCompliantJSONString();
      }

    } else {
      ShapeSimLERR_ << "Missing 'SHAPE' parameters:" << json.getCompliantJSONString();
    }
  } else {
    ShapeSimLERR_ << "Missing 'SHAPE' key:" << json.getCompliantJSONString();
  }
  original_width  = width;
  original_height = height;
  // fetch parameters
  if (shape_params->hasKey("framerate")) {
    framerate = shape_params->getDoubleValue("framerate");
  }
  if (json.hasKey(FRAMEBUFFER_ENCODING_KEY) && json.isStringValue(FRAMEBUFFER_ENCODING_KEY)) {
    pixelEncoding_str = json.getStringValue(FRAMEBUFFER_ENCODING_KEY);
    pixelEncoding     = fmt2cv(pixelEncoding_str);
    ShapeSimLDBG_ << "setting FRAMEBUFFER_ENCODING " << pixelEncoding_str << " :" << pixelEncoding;
  }
  if (shape_params->hasKey("type")) {
    shape_type = shape_params->getStringValue("type");
    ret        = 0;
    centerx    = width / 2;
    centery    = height / 2;
    rotangle   = 0;
    extangle   = 0;
    colr       = 255;
    colg       = 0;
    colb       = 0;
    tickness   = 2;
    linetype   = 8;  // connected
    err_sigmax = err_sigmay = rho = 0;
    GETDBLPARAM(shape_params, movex);
    GETDBLPARAM(shape_params, movey);
    GETDBLPARAM(shape_params, rot);

    GETINTPARAM(shape_params, centerx);
    tmp_centerx = centerx;
    GETINTPARAM(shape_params, centery);
    tmp_centery = centery;

    GETDBLPARAM(shape_params, err_centerx);
    GETDBLPARAM(shape_params, err_centery);
    GETDBLPARAM(shape_params, err_sizex);
    GETDBLPARAM(shape_params, err_sizey);
    GETDBLPARAM(shape_params, err_rotangle);
    GETDBLPARAM(shape_params, err_extangle);
    GETDBLPARAM(shape_params, err_tickness);
    GETINTPARAM(shape_params, sizex);
    tmp_sizex = sizex;
    GETINTPARAM(shape_params, sizey);
    tmp_sizey = sizey;

    GETDBLPARAM(shape_params, rotangle);
    tmp_rotangle = rotangle;

    GETDBLPARAM(shape_params, extangle);

    GETINTPARAM(shape_params, tickness);
    tmp_tickness = tickness;

    GETINTPARAM(shape_params, linetype);
    GETINTPARAM(shape_params, colg);
    GETINTPARAM(shape_params, colb);
    GETINTPARAM(shape_params, colr);
    max_movex     = width - tmp_sizex;
    max_movey     = height - tmp_sizey;
    min_movex     = tmp_sizex;
    min_movey     = tmp_sizey;
    max_amplitude = inc_amplitude = 0;
    GETDBLPARAM(shape_params, max_movex);
    GETDBLPARAM(shape_params, max_movey);
    GETDBLPARAM(shape_params, min_movey);
    GETDBLPARAM(shape_params, min_movex);

    GETDBLPARAM(shape_params, amplitude);
    tmp_amplitude = amplitude;
    max_amplitude = amplitude;
    GETDBLPARAM(shape_params, sigmax);
    GETDBLPARAM(shape_params, sigmay);
    GETDBLPARAM(shape_params, err_sigmay);
    GETDBLPARAM(shape_params, err_sigmax);
    GETDBLPARAM(shape_params, max_amplitude);
    GETDBLPARAM(shape_params, inc_amplitude);
    GETDBLPARAM(shape_params, rho);
  }
  CREATE_INT_PROP("centerx", "", centerx, 0, width, 1);
  CREATE_INT_PROP("centery", "", centery, 0, height, 1);
  CREATE_INT_PROP("sizex", "", sizex, 0, width, 1);
  CREATE_INT_PROP("sizey", "", sizey, 0, height, 1);
  CREATE_DOUBLE_PROP("sigmax", "", sigmax, 0.0, 1.0 * width, 0.1);
  CREATE_DOUBLE_PROP("sigmay", "", sigmay, 0.0, 1.0 * height, 0.1);
  CREATE_DOUBLE_PROP("movex", "", movex, 0.0, 1024.0, 0.1);
  CREATE_DOUBLE_PROP("movey", "", movey, 0.0, 1024.0, 0.1);
  CREATE_DOUBLE_PROP("rot", "", rot, 0.0, 1024.0, 0.01);

  return ret;
}

ShapeSim::ShapeSim()
    : shots(0), frames(0), fn(NULL), pixelEncoding(CV_8UC3), pixelEncoding_str("CV_8UC3"), tmode(CAMERA_TRIGGER_CONTINOUS), gstrategy(CAMERA_LATEST_ONLY), initialized(false), height(CAM_DEFAULT_HEIGTH), width(CAM_DEFAULT_WIDTH), framerate(1.0), offsetx(0), offsety(0), rot(0) {
  ShapeSimLDBG_ << "Created Driver";
  movex = movey = rot = 0;
  max_movex = max_movey = min_movex = min_movey = 0;
  amplitude                                     = 1.0;
  err_amplitude                                 = 0;
  sigmax = sigmay = 10.2;
  gain_raw        = 600;
  brightness_raw  = 500;
  shutter_raw     = 500;
  gain            = 1.0;
  shutter         = 1.0;
  brightness      = 1.0;
  trigger_mode    = 0;
  centerx         = width / 2;
  centery         = height / 2;

  sizex = centerx / 4;
  sizey = centery / 4;

  /* createProperty("ExposureTimeRaw",shutter_raw,0,CAM_MAX_SHUTTER,1,"SHUTTER",[](AbstractCameraDriver*thi,const std::string&name,
     const chaos::common::data::CDataWrapper &p) -> chaos::common::data::CDWUniquePtr {
         // get value
       chaos::common::data::CDWUniquePtr ret(new chaos::common::data::CDataWrapper());
       ret->addInt32Value(PROPERTY_VALUE_KEY,((ShapeSim*)thi)->shutter_raw);
       ShapeSimLDBG_<<"GETTING SHUTTER:"<<((ShapeSim*)thi)->shutter_raw<<" props:"<<p.getJSONString();

       return ret;
     },[](AbstractCameraDriver*thi,const std::string&name,
      const chaos::common::data::CDataWrapper &p) -> chaos::common::data::CDWUniquePtr {
         ((ShapeSim*)thi)->shutter_raw=p.getInt32Value(PROPERTY_VALUE_KEY);
         ShapeSimLDBG_<<"SETTING SHUTTER:"<<((ShapeSim*)thi)->shutter_raw<<" props:"<<p.getJSONString();
         return p.clone();
     });
     */
  CREATE_INT_PROP("SerialNumber", "", serial_id, 0, 0xffffffff, 1);

  CREATE_INT_PROP("ExposureTimeRaw", "SHUTTER", shutter_raw, 0, CAM_MAX_SHUTTER, 1);
  CREATE_INT_PROP("GainRaw", "GAIN", gain_raw, 0, CAM_MAX_GAIN, 1);
  CREATE_INT_PROP("BslBrightnessRaw", "BRIGHTNESS", brightness_raw, 0, CAM_MAX_BRIGHTNESS, 1);
  CREATE_INT_PROP("width", "WIDTH", width, 0, 4096, 1);
  CREATE_INT_PROP("height", "HEIGHT", height, 0, 4096, 1);
  CREATE_INT_PROP("offsetx", "OFFSETX", offsetx, 0, width, 1);
  CREATE_INT_PROP("offsety", "OFFSETY", offsety, 0, height, 1);

  CREATE_INT_PROP("camera_trigger_mode", "TRIGGER_MODE", trigger_mode, 0, 10, 1);

  CREATE_DOUBLE_PROP("framerate", "FRAMERATE", framerate, 0.0, 100.0, 1.0);

  createProperty(
      "PixelFormat", "", FRAMEBUFFER_ENCODING_KEY, [](AbstractDriver* thi, const std::string& name, const chaos::common::data::CDataWrapper& p) -> chaos::common::data::CDWUniquePtr {                    
                   std::string val;
                   ShapeSim *t=(ShapeSim *)thi;
                  chaos::common::data::CDWUniquePtr ret(new chaos::common::data::CDataWrapper());
                  ret->append(PROPERTY_VALUE_KEY,t->pixelEncoding_str);
                  ret->append("raw",t->pixelEncoding_str);
                  ShapeSimLDBG_ << "Pixel Encoding "<<t->pixelEncoding_str<<" :"<<t->pixelEncoding;

                                 
                   return ret; }, [](AbstractDriver* thi, const std::string& name, const chaos::common::data::CDataWrapper& p) -> chaos::common::data::CDWUniquePtr {                    
                
                   
                      ShapeSimLERR_ << "Cannot set Pixel Encoding";
                                                                
                                                                            
                                                         
                   return chaos::common::data::CDWUniquePtr(); });

  /*
      createProperty("GainRaw",gain_raw,0,CAM_MAX_GAIN,1,"GAIN",[](AbstractCameraDriver*thi,const std::string&name,
        const chaos::common::data::CDataWrapper &p) -> chaos::common::data::CDWUniquePtr {
            // get value
          chaos::common::data::CDWUniquePtr ret(new chaos::common::data::CDataWrapper());
          ret->addInt32Value(PROPERTY_VALUE_KEY,((ShapeSim*)thi)->gain_raw);

          return ret;
        },[](AbstractCameraDriver*thi,const std::string&name,
         const chaos::common::data::CDataWrapper &p) -> chaos::common::data::CDWUniquePtr {
            ((ShapeSim*)thi)->gain_raw=p.getInt32Value(PROPERTY_VALUE_KEY);
            return p.clone();
        });
      createProperty("BslBrightnessRaw",brightness_raw,0,CAM_MAX_BRIGHTNESS,1,"BRIGHTNESS",[](AbstractCameraDriver*thi,const std::string&name,
        const chaos::common::data::CDataWrapper &p) -> chaos::common::data::CDWUniquePtr {
            // get value
          chaos::common::data::CDWUniquePtr ret(new chaos::common::data::CDataWrapper());
          ret->addInt32Value(PROPERTY_VALUE_KEY,((ShapeSim*)thi)->brightness_raw);

          return ret;
        },[](AbstractCameraDriver*thi,const std::string&name,
         const chaos::common::data::CDataWrapper &p) -> chaos::common::data::CDWUniquePtr {
            ((ShapeSim*)thi)->brightness_raw=p.getInt32Value(PROPERTY_VALUE_KEY);
            return p.clone();
        });*/

  createProperty(
      "shape_type", "ellipse", "", "", "", "", [](AbstractDriver* thi, const std::string& name, const chaos::common::data::CDataWrapper& p) -> chaos::common::data::CDWUniquePtr {
          // get value
        chaos::common::data::CDWUniquePtr ret(new chaos::common::data::CDataWrapper());
        ret->addStringValue(PROPERTY_VALUE_KEY,((ShapeSim*)thi)->shape_type);

        return ret; }, [](AbstractDriver* thi, const std::string& name, const chaos::common::data::CDataWrapper& p) -> chaos::common::data::CDWUniquePtr { 
          ((ShapeSim*)thi)->shape_type=p.getStringValue(PROPERTY_VALUE_KEY);
          return p.clone(); });
#ifdef CVDEBUG

  cv::namedWindow("test");
#endif
  ShapeSimLDBG_ << " Internal property:" << getProperties()->getJSONString();
}
// default descrutcor
ShapeSim::~ShapeSim() {
}

int ShapeSim::cameraToProps(chaos::common::data::CDataWrapper* p) {
  /*DatasetAttribute speed("speed", "Max speed of trapezoidal profile",
                   chaos::DataType::DataType::TYPE_DOUBLE,
                   chaos::DataType::Input, "0.001", "500.0", "400.0",
                   "0.1", "mm/s");


ShapeSimLDBG_<< "WIDTH:"<<width<<" HEIGHT:"<<height;
ShapeSimLDBG_<< "FRAMERATE:"<<framerate;
p->addDoubleValue("FRAMERATE",framerate);
p->addInt32Value("WIDTH",width);
p->addInt32Value("HEIGHT",height);

p->addInt32Value("TRIGGER_MODE",trigger_mode);
p->addDoubleValue("GAIN",gain);
p->addDoubleValue("BRIGHTNESS",brightness);
p->addDoubleValue("SHUTTER",shutter);
p->addInt32Value("OFFSETX",offsetx);
p->addInt32Value("OFFSETY",offsety);
*/
  syncRead();  // synchronize with real values

  appendPubPropertiesTo(*p);
  ShapeSimLDBG_ << " Properties:" << p->getJSONString();

  return 0;
}
chaos::common::data::CDWUniquePtr ShapeSim::setDrvProperties(chaos::common::data::CDWUniquePtr drv) {
  return setProperties(*drv.get(), true);
}

#define PXL2COLOR(col) \
  if (fmt == #col) {   \
    color = col;       \
  }

int ShapeSim::propsToCamera(chaos::common::data::CDataWrapper* p) {
  // Get the camera control object.
  int32_t ret = 0;

  ShapeSimLDBG_ << "setting props: " << p->getCompliantJSONString();

  // Get the parameters for setting the image area of interest (Image AOI).

  // Maximize the Image AOI.
  //   chaos::common::data::CDataWrapper*p=driver->props;
  setProperties(*p, true);
  /*
  if (p->hasKey("OFFSETX")){
      offsetx=p->getInt32Value("OFFSETX");
      ShapeSimLDBG_<< "setting OFFSETX " << offsetx;

  }
  if (p->hasKey("OFFSETY")){
      offsety= p->getInt32Value("OFFSETY");
      ShapeSimLDBG_<< "setting OFFSET " << offsety;

  }
  if ( p->hasKey("WIDTH")){
      ShapeSimLDBG_<< "setting WIDTH " << p->getInt32Value("WIDTH");
      width=p->getInt32Value("WIDTH");
  }

  if ( p->hasKey("HEIGHT")){
      ShapeSimLDBG_<< "setting HEIGHT " << p->getInt32Value("HEIGHT");
      height=p->getInt32Value("HEIGHT");
  }

   if(p->hasKey("GAIN")){
      gain_raw=p->getDoubleValue("GAIN")*CAM_MAX_GAIN/100;
      ShapeSimLDBG_<< "SETTING GAIN RAW:"<<gain_raw <<" norm:"<<p->getDoubleValue("GAIN");
      gain=p->getDoubleValue("GAIN");
  }

  if(p->hasKey("SHUTTER")){
      shutter_raw=(p->getDoubleValue("SHUTTER")*CAM_MAX_SHUTTER)/100;
      ShapeSimLDBG_<< "SETTING SHUTTER RAW:"<<shutter_raw <<" norm:"<<p->getDoubleValue("SHUTTER");
      shutter=p->getDoubleValue("SHUTTER");
  }
  if(p->hasKey("ZOOM")){

  }
  // Pixel Clock

  if(p->hasKey("FRAMERATE")){
      framerate=p->getDoubleValue("FRAMERATE");
      ShapeSimLDBG_<< "FRAME RATE:"<<framerate;
  }
  if(p->hasKey("TRIGGER_MODE")){
      trigger_mode=p->getInt32Value("TRIGGER_MODE");
  }

  if(p->hasKey("SHARPNESS")){
  }
  if(p->hasKey("BRIGHTNESS")){
      brightness_raw=p->getDoubleValue("BRIGHTNESS")*CAM_MAX_BRIGHTNESS/100;
      ShapeSimLDBG_<< "SETTING BRIGHTNESS RAW:"<<brightness_raw <<" norm:"<<p->getDoubleValue("BRIGHTNESS");
      brightness=p->getDoubleValue("BRIGHTNESS");
  }
  if(p->hasKey("CONTRAST")){
  }



  if (p->hasKey(FRAMEBUFFER_ENCODING_KEY) && p->isStringValue(FRAMEBUFFER_ENCODING_KEY)) {
    ShapeSimLDBG_ << "setting FRAMEBUFFER_ENCODING " << p->getStringValue(FRAMEBUFFER_ENCODING_KEY);

    pixelEncoding = fmt2cv(p->getStringValue(FRAMEBUFFER_ENCODING_KEY));
  }
*/
  return ret;
}

int ShapeSim::cameraInit(void* buffer, uint32_t sizeb) {
  ShapeSimLDBG_ << "Initialization";
  // simple initialization
  frames = 0;

  // For demonstration purposes only, add sample configuration event handlers to print out information
  // about camera use and image grabbing.
  return 0;
}

int ShapeSim::cameraDeinit() {
  ShapeSimLDBG_ << "deinit";
  frames = 0;

  return 0;
}

void ShapeSim::createBeamImage(Size size, Mat& output, float uX, float uY, float sx, float sy, float amp, float angle) {
  Mat    temp = Mat(size, CV_32F);
  double p[6];
  double x[2];

  p[0] = amp;
  p[1] = uX;
  p[2] = sx;
  p[3] = uY;
  p[4] = sy;
  p[5] = angle;

  for (int r = 0; r < size.height; r++) {
    for (int c = 0; c < size.width; c++) {
      x[0]                 = c;
      x[1]                 = r;
      temp.at<float>(r, c) = beamFunc(x, p);
    }
  }
  ShapeSimLDBG_ << "Gaussian(" << uX << "," << uY << ") sx:" << sx << " sy:" << sy << " amp:" << amp;

  normalize(temp, temp, 0.0, 255, NORM_MINMAX);
  if (pixelEncoding == CV_8UC3) {
    temp.convertTo(temp, CV_8U);
    #if (CV_VERSION_MAJOR >= 4)
         cvtColor(temp, output,cv::COLOR_GRAY2BGR);
        #else
    cvtColor(temp, output,  cv::COLOR_GRAY2RGB);
        #endif
  } else if (pixelEncoding == CV_8UC1) {
    temp.convertTo(output, CV_8U);

  } else if (pixelEncoding == CV_16SC1) {
    temp.convertTo(output, CV_16S);

  } else if (pixelEncoding == CV_16UC1) {
    temp.convertTo(output, CV_16U);

  } else {
    throw chaos::CException(-4, "Unsupported Encoding ", __PRETTY_FUNCTION__);
  }
}
int ShapeSim::startGrab(uint32_t _shots, void* _framebuf, cameraGrabCallBack _fn) {
  int ret = -1;
  shots   = _shots;
  // framebuf=_framebuf;
  fn          = _fn;
  err_centerx = 0;
  err_centery = 0;
  err_sizex   = 0;
  err_sizey   = 0;

  err_rotangle = 0;
  err_extangle = 0;
  err_tickness = 0;
  tmp_rotangle = 0;

  last_acquisition_ts = chaos::common::utility::TimingUtil::getTimeStampInMicroseconds();
  ShapeSimLDBG_ << "Start Grabbing at:" << framerate << " frame/s";
  return ret;
}
#define RND_DIST(var)                                                            \
  tmp_##var = var;                                                               \
  if (err_##var > 0) {                                                           \
    boost::random::uniform_int_distribution<> dist_##var(-err_##var, err_##var); \
    double                                    err = dist_##var(gen);             \
    tmp_##var += err;                                                            \
  }

static double getError(double mxerror, boost::random::mt19937& gen) {
  if (mxerror == 0) return 0;
  boost::random::uniform_real_distribution<> dist_err(-mxerror, mxerror);
  return dist_err(gen);
}
void ShapeSim::applyCameraParams(cv::Mat& image) {
  if (gain_raw < 0) {
    gain_raw = CAM_MAX_GAIN;
  }
  if (brightness_raw < 0) {
    brightness_raw = CAM_MAX_BRIGHTNESS;
  }
  if (shutter_raw < 0) {
    shutter_raw = CAM_MAX_SHUTTER;
  }

  for (int y = 0; y < image.rows; y++) {
    for (int x = 0; x < image.cols; x++) {
      if (image.channels() == 1) {
        switch (image.depth()) {
          case CV_8U: {
            float f = (((double)1.0 * gain_raw) * image.at<uchar>(y, x) + (((double)1.0 * brightness_raw)) / CAM_MAX_BRIGHTNESS) * (((double)1.0 * shutter_raw) / CAM_MAX_SHUTTER);

            image.at<uchar>(y, x) = saturate_cast<uchar>(f);
          } break;
          case CV_16S: {
            float f = (((double)1.0 * gain_raw) * image.at<int16_t>(y, x) + (((double)1.0 * brightness_raw)) / CAM_MAX_BRIGHTNESS) * (((double)1.0 * shutter_raw) / CAM_MAX_SHUTTER);

            image.at<int16_t>(y, x) = saturate_cast<int16_t>(f);
            break;
          }
          case CV_16U: {
            float f = (((double)1.0 * gain_raw) * image.at<uint16_t>(y, x) + (((double)1.0 * brightness_raw)) / CAM_MAX_BRIGHTNESS) * (((double)1.0 * shutter_raw) / CAM_MAX_SHUTTER);

            image.at<uint16_t>(y, x) = saturate_cast<uint16_t>(f);
          } break;
          default: {
            float f = (((double)1.0 * gain_raw) * image.at<char>(y, x) + (((double)1.0 * brightness_raw)) / CAM_MAX_BRIGHTNESS) * (((double)1.0 * shutter_raw) / CAM_MAX_SHUTTER);

            image.at<char>(y, x) = saturate_cast<char>(f);
            break;
          }
        }

      } else {
        for (int c = 0; c < image.channels(); c++) {
          float f = (((double)1.0 * gain_raw) / CAM_MAX_GAIN * image.at<Vec3b>(y, x)[c] + (((double)1.0 * brightness_raw)) / CAM_MAX_BRIGHTNESS) * (((double)1.0 * shutter_raw) / CAM_MAX_SHUTTER);

          image.at<Vec3b>(y, x)[c] =
              saturate_cast<uchar>(f);
        }
      }
    }
  }
}

int ShapeSim::waitGrab(camera_buf_t** buf, uint32_t timeout_ms) {
  int32_t                ret = -1;
  size_t                 size_ret;
  int                    size = 0;
  boost::random::mt19937 gen(std::time(0));
  cv::Mat                img = Mat::zeros(original_height, original_width, pixelEncoding);

  // img.zeros(cv::Size(height,width),  CV_8UC3);
  // img.setTo(cv::Scalar::all(0));

  std::stringstream ss, fs;

  ss << getUid() << ":" << frames++;

  if (movex) {
    if (((tmp_centerx + movex) >= max_movex) || ((tmp_centerx + movex) <= min_movex)) {
      movex = -movex;
    }
    tmp_centerx += movex;

  } else {
    RND_DIST(centerx);
    RND_DIST(sizex);
  }

  if (movey) {
    if (((tmp_centery + movey) >= max_movey) || ((tmp_centery + movey) <= min_movey)) {
      movey = -movey;
    }
    tmp_centery += movex;

  } else {
    RND_DIST(centery);
    RND_DIST(sizey);
  }
  if (rot) {
    tmp_rotangle += rot;
  } else {
    RND_DIST(rotangle);
    RND_DIST(tickness);
  }
  if (shape_type == "beam") {
    if (inc_amplitude > 0) {
      if (tmp_amplitude < (amplitude + max_amplitude)) {
        tmp_amplitude += inc_amplitude;
      } else {
        tmp_amplitude = amplitude;
      }

    } else {
      tmp_amplitude = amplitude;
    }
    try {
      createBeamImage(Size(original_width, original_height), img, tmp_centerx + getError((double)err_centerx, gen), tmp_centery + getError((double)err_centery, gen), sigmax + getError(err_sigmax, gen), sigmay + getError(err_sigmay, gen), tmp_amplitude + getError(err_amplitude, gen));
    } catch (cv::Exception e) {
      LERR_ << "## cannot create beam image:" << e.msg;
    }
  } else if (shape_type == "ellipse") {
    // get parameters

    // fs << img.cols << "x" << img.rows << " orig [" << original_width << "x" << original_height << " " << shape_type << ":(" << tmp_centerx << "," << tmp_centery << ") " << tmp_sizex << "x" << tmp_sizey << "," << tmp_rotangle << "," << tmp_tickness;

    rectangle(img, Point(0, 0), Point(width - 1, height - 1), Scalar(colr, colg, colb));

    ellipse(img,
            Point(tmp_centerx, tmp_centery),
            Size(tmp_sizex, tmp_sizey),
            tmp_rotangle,
            0,
            360,
            Scalar(colr, colg, colb),
            tmp_tickness,
            linetype);
    uint64_t ts = chaos::common::utility::TimingUtil::getTimeStamp();

    putText(img, ss.str(), Point(10, 25), FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2, LINE_AA);
    std::string ts_s = chaos::common::utility::TimingUtil::toString(ts, std::string("%H:%M:%S%F"));
    putText(img, ts_s, Point(1, height / 2), FONT_HERSHEY_SIMPLEX, 3, cv::Scalar(255, 255, 255), 3, LINE_AA);

    // putText(img,fs.str(),Point(10,height-25),FONT_HERSHEY_SIMPLEX, 1,(255,255,255),1,LINE_AA);
  } else if (shape_type.size()) {
    if ((access(shape_type.c_str(), F_OK) != -1)) {
      // exist
      try {
        img = imread(shape_type.c_str(), IMREAD_COLOR);
        if (img.empty()) {
          std::cout << "Could not read the image: " << shape_type << std::endl;
          return -5;
        }
      } catch (cv::Exception e) {
        LERR_ << "## cannot create image from file:" << shape_type << " error:" << e.msg;
      } catch (...) {
        LERR_ << "## Exception Reading:" << shape_type;
        return -7;
      }
      // LDBG_<<"Reading:"<<shape_type;
    } else {
      ShapeSimLERR_ << "cannot open:" << shape_type;
      return -3;
    }

  } else {
    ShapeSimLERR_ << "Unknown shape given";
    return -1;
  }
  cv::Mat cropped;
  if (((offsetx + width) < img.cols) && ((offsety + height) < img.rows)) {
    Rect region_of_interest = Rect(offsetx, offsety, width, height);
    Mat  image_roi          = img(region_of_interest);
    image_roi.copyTo(cropped);
    ShapeSimLDBG_ << "Apply ROI:"
                  << "(" << offsetx << "," << offsety << ") " << width << "x" << height << " new image:" << cropped.cols << "x" << cropped.rows;

  } else {
    cropped = img;
  }

  applyCameraParams(cropped);
  size = cropped.total() * cropped.elemSize();

#ifdef CVDEBUG

  cv::imshow("test", img);
  imshow("test", img);
  waitKey(0);
#endif

  if (size > 0 && cropped.data) {
    *buf = new camera_buf_t(cropped.data, size, cropped.cols, cropped.rows);
  } else {
    ShapeSimLERR_ << "BAD BUFFER GIVEN " << shape_type << "(" << width << "X" << height << ")" << frames << " center " << tmp_centerx << "," << tmp_centery << " sizex:" << tmp_sizex << " sizey:" << tmp_sizey << " color:" << colr << "R," << colg << "G," << colb << " size byte:" << size;
    return ret;
  }

  ret = size;

  if (framerate > 0) {
    uint64_t now = chaos::common::utility::TimingUtil::getTimeStampInMicroseconds();

    int32_t diff = (1000000.0 / framerate) - (now - last_acquisition_ts);
    if (diff > 0) {
      // boost::this_thread::sleep_for(boost::chrono::microseconds((uint64_t)diff));
      usleep(diff);
    }
    // ShapeSimLDBG_ << ss.str() << "," << fs.str() << " size byte:" << size << " framerate:" << framerate << " Framebuf encoding:" << pixelEncoding<<" diff:"<<diff;

    (*buf)->ts = last_acquisition_ts = chaos::common::utility::TimingUtil::getTimeStampInMicroseconds();
  }
  /*    if((ret=camera.captureImage(timeout_ms,(char*)framebuf,&size_ret))==0){
      ShapeSimLDBG_<<"Retrieved Image "<<camera.getWidth()<<"x"<<camera.getHeight()<<" raw size:"<<size_ret;
      ret= size_ret;
  } else{
      ShapeSimLERR_<<"No Image..";
  }
*/

  return ret;
}

int ShapeSim::waitGrab(uint32_t timeout_ms) {
  ShapeSimLERR_ << "NOT IMPLEMENTED";

  return 0;
  // return waitGrab((const char**)&framebuf,timeout_ms);
}
int ShapeSim::stopGrab() {
  // camera->StopGrabbing();

  return 0;
}

int ShapeSim::setImageProperties(int32_t width, int32_t height, int32_t opencvImageType) {
  // props->addInt32Value("WIDTH",width);
  // props->addInt32Value("HEIGHT",height);
  // props->addDoubleValue("FRAMERATE",framerate);
  setPropertyValue("WIDTH", width, true);
  setPropertyValue("HEIGHT", height, true);

  return 0;
}

int ShapeSim::getImageProperties(int32_t& width, int32_t& height, int32_t& opencvImageType) {
  ChaosUniquePtr<chaos::common::data::CDataWrapper> cw(new chaos::common::data::CDataWrapper());
  int                                               ret = -1;
  getProperty("WIDTH", width, true);
  getProperty("HEIGHT", height, true);

  /* cameraToProps(cw.get());
   if(cw->hasKey("WIDTH")){
       width=cw->getInt32Value("WIDTH");
       ret=0;
   }

   if(cw->hasKey("HEIGHT")){
       height=cw->getInt32Value("HEIGHT");
       ret++;
   }
*/
  return (ret > 0) ? 0 : ret;
}

int ShapeSim::setCameraProperty(const std::string& propname, int32_t val) {
  ChaosUniquePtr<chaos::common::data::CDataWrapper> cw(new chaos::common::data::CDataWrapper());
  ShapeSimLDBG_ << "Setting \"" << propname << "\"=" << (int32_t)val;

  cw->addInt32Value(propname, (int32_t)val);

  return propsToCamera(cw.get());
}
int ShapeSim::setCameraProperty(const std::string& propname, double val) {
  ChaosUniquePtr<chaos::common::data::CDataWrapper> cw(new chaos::common::data::CDataWrapper());
  ShapeSimLDBG_ << "Setting \"" << propname << "\"=" << val;

  cw->addDoubleValue(propname, val);

  return propsToCamera(cw.get());
}

int ShapeSim::getCameraProperty(const std::string& propname, int32_t& val) {
  ChaosUniquePtr<chaos::common::data::CDataWrapper> cw(new chaos::common::data::CDataWrapper());

  cameraToProps(cw.get());

  if (cw->hasKey(propname)) {
    val = cw->getInt32Value(propname);
    return 0;
  }
  return -1;
}
int ShapeSim::cameraRoi(int sizex,int sizey,int x, int y){
  ShapeSimLDBG_ << " performing ROI ("<<width<<"->"<<sizex<<") offx ("<<offsetx<<"->"<<x<<") "<< " ("<<height<<"->"<<sizey<<") offy ("<<offsety<<"->"<<y<<")";

  if((sizex>original_width)||(sizex<0)){
    width=original_width;

  }
  if((sizey>original_height)||(sizey<0)){
    height=original_height;
  }
  if((x>original_width)||(x<0)){
    offsetx=0;
  }
  if((x>original_height)||(y<0)){
    offsety=0;
  }
  if(sizey>original_height){
    height=original_height;
  }
  if(sizex+x<original_width){
    width=sizex;
    offsetx=x;
  } else {
      ShapeSimLERR_ << " cannot perform ROIX ("<<width<<"->"<<sizex<<") offx ("<<offsetx<<"->"<<x<<")" ;

    return -1;
  }
  if(sizey+y<original_height){
    height=sizey;
    offsety=y;
  } else {
      ShapeSimLERR_ << " cannot perform ROIY ("<<height<<"->"<<sizey<<") offy ("<<offsety<<"->"<<y<<")" ;

    return -2;
  }
  
  return 0;
}

int ShapeSim::getCameraProperty(const std::string& propname, double& val) {
  ChaosUniquePtr<chaos::common::data::CDataWrapper> cw(new chaos::common::data::CDataWrapper());

  cameraToProps(cw.get());
  if (cw->hasKey(propname)) {
    val = cw->getDoubleValue(propname);
    return 0;
  }
  return -1;
}

int ShapeSim::getCameraProperties(chaos::common::data::CDataWrapper& proplist) {
  return cameraToProps(&proplist);
}

}  // namespace camera
}  // namespace sensor
}  // namespace driver
