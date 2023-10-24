/*
 *	MemCacheCam.h
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
#include "MemCacheCam.h"
#include <chaos/common/data/CDataWrapper.h>
#include <chaos/cu_toolkit/driver_manager/driver/AbstractDriverPlugin.h>
#include <math.h>
#include <stdlib.h>
//#include <boost/algorithm/string.hpp>
//#include <boost/lexical_cast.hpp>
#include <chaos/common/ChaosCommon.h>
#include <string>
#include <chaos/common/utility/endianess.h>

#include <chaos/common/data/structured/DatasetAttribute.h>

namespace cu_driver = chaos::cu::driver_manager::driver;
using namespace ::driver::sensor::camera;
using namespace chaos::common::data::structured;

using namespace cv;

#define MemCacheCamLAPP_ LAPP_ << "[MemCacheCam] "
#define MemCacheCamLDBG_ LDBG_ << "[MemCacheCam:" << __PRETTY_FUNCTION__ << "]"
#define MemCacheCamLERR_ LERR_ << "[MemCacheCam:" << __PRETTY_FUNCTION__ << "]"
#define MemcachedDataImporterDriverLOG_MC_ERROR(x)        \
  if (mc_result != MEMCACHED_SUCCESS) {                   \
    MemCacheCamLERR_ << memcached_strerror(mc_client, x); \
    return -10;                                           \
  }

#define CREATE_INT_PROP(n, pub, var, min, max, inc) \
  createProperty(                                   \
      n, var, (int32_t)min, (int32_t)max, (int32_t)inc, pub, [](AbstractDriver* thi, const std::string& name, const chaos::common::data::CDataWrapper& p) -> chaos::common::data::CDWUniquePtr {\
        chaos::common::data::CDWUniquePtr ret(new chaos::common::data::CDataWrapper());\
        ret->addInt32Value(PROPERTY_VALUE_KEY,((MemCacheCam*)thi)->var);\
        return ret; }, [](AbstractDriver* thi, const std::string& name, const chaos::common::data::CDataWrapper& p) -> chaos::common::data::CDWUniquePtr { \
          ((MemCacheCam*)thi)->var=p.getInt32Value(PROPERTY_VALUE_KEY);\
          return p.clone(); });

#define CREATE_DOUBLE_PROP(n, pub, var, min, max, inc) \
  createProperty(                                      \
      n, var, min, max, inc, pub, [](AbstractDriver* thi, const std::string& name, const chaos::common::data::CDataWrapper& p) -> chaos::common::data::CDWUniquePtr {\
        chaos::common::data::CDWUniquePtr ret(new chaos::common::data::CDataWrapper());\
        ret->addDoubleValue(PROPERTY_VALUE_KEY,((MemCacheCam*)thi)->var);\
        return ret; }, [](AbstractDriver* thi, const std::string& name, const chaos::common::data::CDataWrapper& p) -> chaos::common::data::CDWUniquePtr { \
          ((MemCacheCam*)thi)->var=p.getDoubleValue(PROPERTY_VALUE_KEY);\
          return p.clone(); });

#define GETDBLPARAM(w, name)                                           \
  if (w->hasKey(#name)) {                                              \
    name = w->getDoubleValue(#name);                                   \
    MemCacheCamLDBG_ << "shape DBL param '" << #name << " = " << name; \
  }
#define GETINTPARAM(w, name)                                           \
  if (w->hasKey(#name)) {                                              \
    name = w->getInt32Value(#name);                                    \
    MemCacheCamLDBG_ << "shape INT param '" << #name << " = " << name; \
  }

// GET_PLUGIN_CLASS_DEFINITION
// we need only to define the driver because we don't are makeing a plugin
OPEN_CU_DRIVER_PLUGIN_CLASS_DEFINITION(MemCacheCam, 1.0.0, ::driver::sensor::camera::MemCacheCam)
REGISTER_CU_DRIVER_PLUGIN_CLASS_INIT_ATTRIBUTE(::driver::sensor::camera::MemCacheCam, http_address / dnsname
                                               : port)
CLOSE_CU_DRIVER_PLUGIN_CLASS_DEFINITION
OPEN_REGISTER_PLUGIN
REGISTER_PLUGIN(::driver::sensor::camera::MemCacheCam)
CLOSE_REGISTER_PLUGIN


MemCacheCam::MemCacheCam()
    : fn(NULL),bigendian(1), gain_raw(1.0),tmode(CAMERA_TRIGGER_CONTINOUS), pixelEncoding(CV_8UC3), pixelEncoding_str("CV_8UC3"), height(CAM_DEFAULT_HEIGTH), width(CAM_DEFAULT_WIDTH), framerate(1.0), offsetx(0), offsety(0), mc_client(NULL) {
  MemCacheCamLDBG_ << "Created Driver";
  raw_sizex=raw_sizey=raw_offset=-1;

  brightness_raw = 0;
  shutter_raw    = 1;
  serial_id      = time(NULL);
  CREATE_INT_PROP("SerialNumber", "", serial_id, 0, 0xffffffff, 1);
  CREATE_INT_PROP("BigEndian", "", bigendian, 0, 1, 1);

  CREATE_INT_PROP("ExposureTimeRaw", "SHUTTER", shutter_raw, 0, CAM_MAX_SHUTTER, 1);
  CREATE_INT_PROP("BslBrightnessRaw", "BRIGHTNESS", brightness_raw, 0, CAM_MAX_BRIGHTNESS, 1);
  CREATE_INT_PROP("width", "WIDTH", width, 0, 4096, 1);
  CREATE_INT_PROP("height", "HEIGHT", height, 0, 4096, 1);
  CREATE_INT_PROP("offsetx", "OFFSETX", offsetx, 0, width, 1);
  CREATE_INT_PROP("offsety", "OFFSETY", offsety, 0, height, 1);
  CREATE_DOUBLE_PROP("framerate", "FRAMERATE", framerate, 0.0, 100.0, 1.0);
  CREATE_INT_PROP("camera_trigger_mode", "TRIGGER_MODE", tmode, 0, 10, 1);
  CREATE_INT_PROP("ExposureTimeRaw", "SHUTTER", shutter_raw, 0, CAM_MAX_SHUTTER, 1);
  CREATE_DOUBLE_PROP("GainRaw", "GAIN", gain_raw, 0.0, 100.0, 0.1);
  createProperty(
      "PixelFormat", "", FRAMEBUFFER_ENCODING_KEY, [](AbstractDriver* thi, const std::string& name, const chaos::common::data::CDataWrapper& p) -> chaos::common::data::CDWUniquePtr {                    
                   std::string val;
                   MemCacheCam *t=(MemCacheCam *)thi;
                  chaos::common::data::CDWUniquePtr ret(new chaos::common::data::CDataWrapper());
                  ret->append(PROPERTY_VALUE_KEY,t->pixelEncoding_str);
                  ret->append("raw",t->pixelEncoding_str);

                  MemCacheCamLDBG_ << "Pixel Encoding "<<t->pixelEncoding_str<<" :"<<t->pixelEncoding;

                                 
                   return ret; }, [](AbstractDriver* thi, const std::string& name, const chaos::common::data::CDataWrapper& p) -> chaos::common::data::CDWUniquePtr {                    
                
                   
                      MemCacheCamLERR_ << "Cannot set Pixel Encoding";
                                                                
                                                                            
                                                         
                   return chaos::common::data::CDWUniquePtr(); });

#ifdef CVDEBUG

  cv::namedWindow("test");
#endif
  MemCacheCamLDBG_ << " Internal property:" << getProperties()->getJSONString();
}
// default descrutcor
MemCacheCam::~MemCacheCam() {
}

void MemCacheCam::driverInit(const char* initParameter)  {
  throw chaos::CException(-1, "cannot intialize camera expected bad JSON parameters" + std::string(initParameter), __PRETTY_FUNCTION__);
}

void MemCacheCam::driverInit(const chaos::common::data::CDataWrapper& json)  {
  MemCacheCamLDBG_ << "Initializing  driver:" << json.getCompliantJSONString();
  MemCacheCamLDBG_ << "Inital properties" << getProperties()->getJSONString();
  if (initializeCamera(json) != 0) {
    throw chaos::CException(-1, "cannot initialize camera " + json.getCompliantJSONString(), __PRETTY_FUNCTION__);
  }
}

void MemCacheCam::driverDeinit()  {
  MemCacheCamLAPP_ << "Deinit driver";
  if (mc_client) {
    memcached_free(mc_client);
    mc_client = NULL;
  }
}

namespace driver {

namespace sensor {
namespace camera {

// GET_PLUGIN_CLASS_DEFINITION
// we need to define the driver with alias version and a class that implement it
int MemCacheCam::initializeCamera(const chaos::common::data::CDataWrapper& json) {
  int ret = -2;
  MemCacheCamLDBG_ << "intialize camera:" << json.getCompliantJSONString();
  propsToCamera((chaos::common::data::CDataWrapper*)&json);
  if (json.hasKey("KEY")) {
    key = json.getStringValue("KEY");
  } else {
    MemCacheCamLERR_ << "Missing 'KEY'" << json.getCompliantJSONString();
    return ret;
  }
  if (json.hasKey("rawsizex")) {
    raw_sizex = json.getInt32Value("rawsizex");
  } 
  if (json.hasKey("rawsizey")) {
    raw_sizey = json.getInt32Value("rawsizey");
  }
  if (json.hasKey("rawoffset")) {
    raw_offset = json.getInt32Value("rawoffset");
  }
  

  if (json.hasKey("SERVER")) {
    server = json.getStringValue("SERVER");
  } else {
    MemCacheCamLERR_ << "Missing 'SERVER'" << json.getCompliantJSONString();
    return ret;
  }
  if (json.hasKey("BE")) {
    bigendian = json.getInt32Value("BE");
  }
  
  /* if (json.hasKey("HEIGHT")) {
     height=json.getInt32Value("HEIGHT");

   } else {
     MemCacheCamLERR_ << "Missing 'HEIGHT'" << json.getCompliantJSONString();
     return ret;
   }
   if (json.hasKey("WIDTH")) {
     width=json.getInt32Value("WIDTH");
   } else {
     MemCacheCamLERR_ << "Missing 'WIDTH'" << json.getCompliantJSONString();
     return ret;
   }*/
  if (json.hasKey(FRAMEBUFFER_ENCODING_KEY) && json.isStringValue(FRAMEBUFFER_ENCODING_KEY)) {
    pixelEncoding_str = json.getStringValue(FRAMEBUFFER_ENCODING_KEY);
    pixelEncoding     = fmt2cv(pixelEncoding_str);
    MemCacheCamLDBG_ << "setting FRAMEBUFFER_ENCODING " << pixelEncoding_str << " :" << pixelEncoding;
  } else {
    MemCacheCamLERR_ << "Missing " << FRAMEBUFFER_ENCODING_KEY << " :" << json.getCompliantJSONString();
    return ret;
  }
  original_width  = width;
  original_height = height;

  mc_client = memcached(NULL, 0);
  if (mc_client == NULL) {
    MemCacheCamLERR_ << "cannot allocate memcache client";

    return -4;
  }
  memcached_return_t       mc_result;
  std::vector<std::string> host_port_vec;
 // boost::split(host_port_vec, server, boost::is_any_of(":"));
host_port_vec=chaos::split(server,":");
  if (host_port_vec.size() == 1) {
    host_port_vec.push_back("11211");
  }
  if ((mc_result = memcached_server_add(
           mc_client, host_port_vec[0].c_str(), boost::lexical_cast<in_port_t>(host_port_vec[1]))) !=
      MEMCACHED_SUCCESS) {
    MemcachedDataImporterDriverLOG_MC_ERROR(mc_result);
  }

  MemcachedDataImporterDriverLOG_MC_ERROR(
      mc_result = memcached_behavior_set(
          mc_client, MEMCACHED_BEHAVIOR_BINARY_PROTOCOL, (uint64_t)1))
      MemcachedDataImporterDriverLOG_MC_ERROR(
          mc_result = memcached_behavior_set(
              mc_client, MEMCACHED_BEHAVIOR_NO_BLOCK, (uint64_t)1))
          MemcachedDataImporterDriverLOG_MC_ERROR(
              mc_result = memcached_behavior_set(
                  mc_client, MEMCACHED_BEHAVIOR_NOREPLY, (uint64_t)1))
              MemcachedDataImporterDriverLOG_MC_ERROR(
                  mc_result = memcached_behavior_set(
                      mc_client, MEMCACHED_BEHAVIOR_KETAMA, (uint64_t)1))
                  MemcachedDataImporterDriverLOG_MC_ERROR(
                      mc_result = memcached_behavior_set(
                          mc_client, MEMCACHED_BEHAVIOR_TCP_NODELAY, (uint64_t)1))
                      MemcachedDataImporterDriverLOG_MC_ERROR(
                          mc_result = memcached_behavior_set(
                              mc_client,
                              MEMCACHED_BEHAVIOR_SERVER_FAILURE_LIMIT,
                              (uint64_t)2))
                          MemcachedDataImporterDriverLOG_MC_ERROR(
                              mc_result = memcached_behavior_set(
                                  mc_client,
                                  MEMCACHED_BEHAVIOR_REMOVE_FAILED_SERVERS,
                                  (uint64_t)1))
                              MemcachedDataImporterDriverLOG_MC_ERROR(
                                  mc_result = memcached_behavior_set(
                                      mc_client,
                                      MEMCACHED_BEHAVIOR_RETRY_TIMEOUT,
                                      (uint64_t)2))
                                  MemcachedDataImporterDriverLOG_MC_ERROR(
                                      mc_result = memcached_behavior_set(
                                          mc_client,
                                          MEMCACHED_BEHAVIOR_CONNECT_TIMEOUT,
                                          (uint64_t)500))
                                      MemcachedDataImporterDriverLOG_MC_ERROR(
                                          mc_result = memcached_behavior_set(
                                              mc_client,
                                              MEMCACHED_BEHAVIOR_TCP_KEEPALIVE,
                                              (uint64_t)1)) return 0;
}


int MemCacheCam::cameraToProps(chaos::common::data::CDataWrapper* p) {
  syncRead();  // synchronize with real values

  appendPubPropertiesTo(*p);
  MemCacheCamLDBG_ << " Properties:" << p->getJSONString();

  return 0;
}
chaos::common::data::CDWUniquePtr MemCacheCam::setDrvProperties(chaos::common::data::CDWUniquePtr drv) {
  return setProperties(*drv.get(), true);
}

#define PXL2COLOR(col) \
  if (fmt == #col) {   \
    color = col;       \
  }

int MemCacheCam::propsToCamera(chaos::common::data::CDataWrapper* p) {
  // Get the camera control object.
  int32_t ret = 0;

  MemCacheCamLDBG_ << "setting props: " << p->getCompliantJSONString();

  // Get the parameters for setting the image area of interest (Image AOI).

  // Maximize the Image AOI.
  //   chaos::common::data::CDataWrapper*p=driver->props;
  setProperties(*p, true);

  return ret;
}

int MemCacheCam::cameraInit(void* buffer, uint32_t sizeb) {
  MemCacheCamLDBG_ << "Initialization";
  // simple initialization

  // For demonstration purposes only, add sample configuration event handlers to print out information
  // about camera use and image grabbing.
  return 0;
}

int MemCacheCam::cameraDeinit() {
  MemCacheCamLDBG_ << "deinit";

  return 0;
}

int MemCacheCam::startGrab(uint32_t _shots, void* _framebuf, cameraGrabCallBack _fn) {
  int ret = -1;
  shots   = _shots;
  // framebuf=_framebuf;
  fn = _fn;

  last_acquisition_ts = chaos::common::utility::TimingUtil::getTimeStampInMicroseconds();
  MemCacheCamLDBG_ << "Start Grabbing at:" << framerate << " frame/s";
  pixelEncoding     = fmt2cv(pixelEncoding_str);

  return ret;
}
#define RND_DIST(var)                                                            \
  tmp_##var = var;                                                               \
  if (err_##var > 0) {                                                           \
    boost::random::uniform_int_distribution<> dist_##var(-err_##var, err_##var); \
    double                                    err = dist_##var(gen);             \
    tmp_##var += err;                                                            \
  }

void MemCacheCam::applyCameraParams(cv::Mat& image) {
  if (gain_raw < 0) {
    gain_raw = 1.0;
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

int MemCacheCam::waitGrab(camera_buf_t** buf, uint32_t timeout_ms) {
  int32_t                ret = -1;
  size_t                 size_ret;
  int                    size  = 0;
  uint32_t               flags = 0;
  memcached_return_t     rc;
  boost::random::mt19937 gen(std::time(0));
  //cv::Mat                img = Mat::zeros(original_height, original_width, pixelEncoding);

  // img.zeros(cv::Size(height,width),  CV_8UC3);
  // img.setTo(cv::Scalar::all(0));
  size_t value_length = 0;
  uint64_t now        = chaos::common::utility::TimingUtil::getTimeStampInMicroseconds();

  if (framerate > 0) {
    double   diff       = (1000000.0 / framerate) - (now - last_acquisition_ts);
    last_acquisition_ts = now;
    if (diff > 0) {
      boost::this_thread::sleep_for(boost::chrono::microseconds((uint64_t)diff));
    }
  }
  char* value = memcached_get(mc_client, key.c_str(), key.size()+1, &value_length, &flags, &rc);
  // check if we have something
  if ((value != NULL) && (rc == MEMCACHED_SUCCESS) && (value_length > 2 * sizeof(uint32_t))) {
    // we expect a 2D array first word= # of rows, second word= # number of column
    char*ptrbuf=value;
    if(raw_offset>0){
      ptrbuf+=raw_offset;
    }

    uint32_t* ptr = (uint32_t*)ptrbuf;
    
    int siz=value_length; 
     uint16_t*src16=( uint16_t*)ptr;
     int16_t*ssrc16=( int16_t*)ptr;
     uint8_t*src8=( uint8_t*)ptr;
    if((raw_sizex>0)&& (raw_sizey>0)){
      width=raw_sizex;
      height=raw_sizey;
    } else {
      siz-= 2 * sizeof(uint32_t);
      if(bigendian){
        src16=( uint16_t*)&ptr[2];
        ssrc16=( int16_t*)&ptr[2];
        src8=( uint8_t*)&ptr[2];
        width         = chaos::common::utility::byte_swap<chaos::common::utility::host_endian,
                                                                              chaos::common::utility::big_endian, uint32_t>(ptr[1]);  // cols
        height        = chaos::common::utility::byte_swap<chaos::common::utility::host_endian,
       
                                                                      chaos::common::utility::big_endian, uint32_t>(ptr[0]);  // row
      } else {
        width=ptr[1];
        height=ptr[0];
      }
    }
    
    *buf          = new camera_buf_t(siz*sizeof(uint16_t) , width, height);
    if((pixelEncoding==CV_16UC1)||(pixelEncoding==CV_16SC1)){
      
      uint16_t*pu=(uint16_t*)(*buf)->buf;
      int16_t*pus=(int16_t*)(*buf)->buf;

      for(int cnt=0;cnt<siz/sizeof(uint16_t);cnt++){
        if(bigendian){
          if(pixelEncoding==CV_16SC1){

            int16_t val=chaos::common::utility::byte_swap<chaos::common::utility::host_endian,
                                                                             chaos::common::utility::big_endian, int16_t>(ssrc16[cnt]);
                                                                           
            pus[cnt]=val* gain_raw + brightness_raw;
      //    printf("%d ",pu[cnt]);
          } else {

            uint16_t val=chaos::common::utility::byte_swap<chaos::common::utility::host_endian,
                                                                             chaos::common::utility::big_endian, uint16_t>(src16[cnt]);
                                                                           
            pu[cnt]=val* gain_raw + brightness_raw;
      //    printf("%d ",pu[cnt]);
          } 
        } else {
           if(pixelEncoding==CV_16SC1){
            pus[cnt]=ssrc16[cnt]* gain_raw + brightness_raw;

           } else {
            pu[cnt]=src16[cnt]* gain_raw + brightness_raw;
           }
        }
      }
    //  printf("\n");
        MemCacheCamLDBG_ << "Acquired \"" << key << "\" size:" << value_length << " " << width << "x" << height << " encoding:" << pixelEncoding_str<<"("<<pixelEncoding<<")";

    }
    if((pixelEncoding==CV_8UC1)||(pixelEncoding==CV_8SC1)){
      
      uint8_t max=0,min=255;

      uint8_t*pu=(uint8_t*)(*buf)->buf;
      for(int cnt=0;cnt<siz;cnt++){
          pu[cnt]=src8[cnt]* gain_raw + brightness_raw;
          max=std::max(pu[cnt],max);
          min=std::min(pu[cnt],min);

      }
        MemCacheCamLDBG_ << "Acquired \"" << key << "\" size:" << value_length << " " << width << "x" << height << " encoding:" << pixelEncoding_str<<"("<<pixelEncoding<<")" <<" MAX:"<<max<<" MIN:"<<min;

    }
    (*buf)->ts = chaos::common::utility::TimingUtil::getTimeStampInMicroseconds();

    free(value);
    return siz;
  } else {
    MemCacheCamLERR_ << "Error retriving data from key "
                     << key << " server:"<<server<<" memcod ret:" << rc << " :" << memcached_strerror(mc_client, rc);
    return rc;
  }

  return 0;
}

int MemCacheCam::waitGrab(uint32_t timeout_ms) {
  MemCacheCamLERR_ << "NOT IMPLEMENTED";

  return 0;
  // return waitGrab((const char**)&framebuf,timeout_ms);
}
int MemCacheCam::stopGrab() {
  // camera->StopGrabbing();

  return 0;
}

int MemCacheCam::setImageProperties(int32_t width, int32_t height, int32_t opencvImageType) {
  // props->addInt32Value("WIDTH",width);
  // props->addInt32Value("HEIGHT",height);
  // props->addDoubleValue("FRAMERATE",framerate);
  setPropertyValue("WIDTH", width, true);
  setPropertyValue("HEIGHT", height, true);

  return 0;
}

int MemCacheCam::getImageProperties(int32_t& width, int32_t& height, int32_t& opencvImageType) {
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

int MemCacheCam::setCameraProperty(const std::string& propname, int32_t val) {
  ChaosUniquePtr<chaos::common::data::CDataWrapper> cw(new chaos::common::data::CDataWrapper());
  MemCacheCamLDBG_ << "Setting \"" << propname << "\"=" << (int32_t)val;

  cw->addInt32Value(propname, (int32_t)val);

  return propsToCamera(cw.get());
}
int MemCacheCam::setCameraProperty(const std::string& propname, double val) {
  ChaosUniquePtr<chaos::common::data::CDataWrapper> cw(new chaos::common::data::CDataWrapper());
  MemCacheCamLDBG_ << "Setting \"" << propname << "\"=" << val;

  cw->addDoubleValue(propname, val);

  return propsToCamera(cw.get());
}

int MemCacheCam::getCameraProperty(const std::string& propname, int32_t& val) {
  ChaosUniquePtr<chaos::common::data::CDataWrapper> cw(new chaos::common::data::CDataWrapper());

  cameraToProps(cw.get());

  if (cw->hasKey(propname)) {
    val = cw->getInt32Value(propname);
    return 0;
  }
  return -1;
}

int MemCacheCam::getCameraProperty(const std::string& propname, double& val) {
  ChaosUniquePtr<chaos::common::data::CDataWrapper> cw(new chaos::common::data::CDataWrapper());

  cameraToProps(cw.get());
  if (cw->hasKey(propname)) {
    val = cw->getDoubleValue(propname);
    return 0;
  }
  return -1;
}
int MemCacheCam::cameraRoi(int sizex,int sizey,int x, int y){
  return -1;
}

int MemCacheCam::getCameraProperties(chaos::common::data::CDataWrapper& proplist) {
  return cameraToProps(&proplist);
}

}  // namespace camera
}  // namespace sensor
}  // namespace driver
