/*
 *	IDSGEXXDriver.h
 *	!CHAOS
 *	Created by Andrea Michelotti.
 *
 *    	Copyright 2013 INFN, National Institute of Nuclear Physics
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
#include "IDSGEXXDriver.h"
#include "Camera.h"

#include <chaos/common/data/CDataWrapper.h>
#include <chaos/cu_toolkit/driver_manager/driver/AbstractDriverPlugin.h>
#include <math.h>
#include <stdlib.h>
#include <string>

// use the pylon driver

namespace cu_driver = chaos::cu::driver_manager::driver;
using namespace ::driver::sensor::camera;
#define IDSGEXXDriverLAPP_ LAPP_ << "[IDSGEXXDriver-" IDSVER "] -" << serial << "-"
#define IDSGEXXDriverLDBG_                                                     \
  LDBG_ << "[IDSGEXXDriver-" IDSVER " " << __FUNCTION__ << "]-" << serial << "-"
#define IDSGEXXDriverLDBG(t)                                                   \
  LDBG_ << "[IDSGEXXDriver-" IDSVER " " << __FUNCTION__ << "]-" << t->serial << "-"

#define IDSGEXXDriverLERR_                                                     \
  LERR_ << "[IDSGEXXDriver-" IDSVER " "<< __PRETTY_FUNCTION__ << "]-" << serial << "-"
#define IDSGEXXDriverLERR(t)                                                   \
  LERR_ << "[IDSGEXXDriver-" IDSVER " "<< __PRETTY_FUNCTION__ << "]-" << t->serial << "-"
// GET_PLUGIN_CLASS_DEFINITION
// we need only to define the driver because we don't are makeing a plugin
OPEN_CU_DRIVER_PLUGIN_CLASS_DEFINITION(IDSGEXXDriver, 1.0.0,::driver::sensor::camera::IDSGEXXDriver)
REGISTER_CU_DRIVER_PLUGIN_CLASS_INIT_ATTRIBUTE(
    ::driver::sensor::camera::IDSGEXXDriver, http_address / dnsname
    : port)
CLOSE_CU_DRIVER_PLUGIN_CLASS_DEFINITION
OPEN_REGISTER_PLUGIN
REGISTER_PLUGIN(::driver::sensor::camera::IDSGEXXDriver)
CLOSE_REGISTER_PLUGIN

using namespace ueye;
int32_t IDStrgmode2trgmode(ueye::TriggerMode ids) {
  switch (ids) {
  case TRIGGER_HI_LO:
//    IDSGEXXDriverLDBG_ << "TRIGGER HI->LOW";
    return CAMERA_TRIGGER_HW_LOW;
  case TRIGGER_LO_HI:
 //   IDSGEXXDriverLDBG_ << "TRIGGER LOW->HI";
    return CAMERA_TRIGGER_HW_HI;
  case TRIGGER_OFF:

  default:
  //  IDSGEXXDriverLDBG_ << "TRIGGER CONTINOUS";
    return CAMERA_TRIGGER_CONTINOUS;
  }
}
TriggerMode trgmode2IDStrgmode(int32_t tmode) {
  switch (tmode) {
  case (CAMERA_TRIGGER_CONTINOUS): {
  //  IDSGEXXDriverLDBG_ << "TRIGGER OFF";

    return TRIGGER_OFF;
  }
  case CAMERA_TRIGGER_SINGLE: {
 //   IDSGEXXDriverLDBG_ << "TRIGGER SINGLE (NA)";
    return TRIGGER_LO_HI;
  }
  case CAMERA_TRIGGER_SOFT: {
 //   IDSGEXXDriverLDBG_ << "TRIGGER SOFT (NA)";
    return TRIGGER_LO_HI;
  }

  case CAMERA_TRIGGER_HW_LOW:
  //  IDSGEXXDriverLDBG_ << "TRIGGER HI->LOW";

    return TRIGGER_HI_LO;

  case CAMERA_TRIGGER_HW_HI:
  //  IDSGEXXDriverLDBG_ << "TRIGGER LOW->HI";
    return TRIGGER_LO_HI;
  default:
  //  IDSGEXXDriverLDBG_ << "DISABLE ACQUIRE";
    return TRIGGER_OFF;
  }
}
static std::string ids2cv(uEyeColor fmt) {
  switch (fmt) {
  case MONO8:
    return "CV_8UC1";
  case MONO16:
    return "CV_16UC1";
  case RGB8:
  case BGR8:
    return "CV_8UC3";
  default:
    return "NOT SUPPORTED";
  }
  return "NOT SUPPORTED";
}
static uEyeColor cv2ids(const std::string &fmt) {
  if (fmt == "CV_8UC1")
    return MONO8;
  if (fmt == "CV_8SC1")
    return MONO8;
  if (fmt == "CV_16UC1")
    return MONO16;
  if (fmt == "CV_8UC3")
    return RGB8;
  return MONO8;
}
void IDSGEXXDriver::driverInit(const char *initParameter) {
  throw chaos::CException(
      -3, "You should provide a valid JSON initialization string",
      __PRETTY_FUNCTION__);
}
/*
int IDSGEXXDriver::get_next_image(char **mem, INT *image_id, int32_t timeout) {
  int ret;

  ret = is_WaitForNextImage(hCam, timeout, mem, image_id);

  switch (ret) {
  case IS_SUCCESS:
    break;
  case IS_TIMED_OUT:
    IDSGEXXDriverLERR_ << "Timeout exceeded:" << timeout;
    return -1;
  case IS_CAPTURE_STATUS:
    IDSGEXXDriverLERR_ << "Transfer error.  Check capture status." << timeout;

    return -2;
  default:
    return -3;
  }

  return 0;
}
*/
void IDSGEXXDriver::driverInit(const chaos::common::data::CDataWrapper &json) {
  IDSGEXXDriverLAPP_ << "Initializing  json driver:"
                     << json.getCompliantJSONString();
  parseInitCommonParams(json);

  if (initializeCamera(json) != 0) {
    throw chaos::CException(
        -1, "cannot initialize camera " + json.getCompliantJSONString(),
        __PRETTY_FUNCTION__);
  }
}

void IDSGEXXDriver::driverDeinit() {

  IDSGEXXDriverLAPP_ << "Deinit driver";
  camera->closeCamera();
}

namespace driver {

namespace sensor {
namespace camera {

// GET_PLUGIN_CLASS_DEFINITION
// we need to define the driver with alias version and a class that implement it
int IDSGEXXDriver::initializeCamera(
    const chaos::common::data::CDataWrapper &json) {
  int ret;
  const char *version;
  int major, minor, build;
  grabbing = false;
  try {
    /* if (camera->checkVersion(major, minor, build, version)) {
         IDSGEXXDriverLDBG_<<"Loaded uEye SDK:"<<version;
     } else {
         IDSGEXXDriverLERR_<<"failed to check version!";
     }
 */
    // Make sure there is at least one camera available
    int num_cameras = camera->getNumberOfCameras();
    if (num_cameras > 0) {
      IDSGEXXDriverLDBG_ << "Found " << num_cameras << " cameras";
      std::vector<unsigned int> serialid, dev_id;

      int ret = camera->getSerialNumberList(serialid, dev_id);
      for (int cnt = 0; cnt < ret; cnt++) {
        IDSGEXXDriverLDBG_ << cnt << "] Found serid:" << serialid[cnt]
                           << " devid:" << dev_id[cnt] << " looking:" << serial;
      }

    } else {
      IDSGEXXDriverLERR_ << " NO CAMERA FOUND";
      return -1;
    }
    if (serial.size()) {
      uint32_t id = atoi(serial.c_str());
      IDSGEXXDriverLDBG_ << "opening serial:" << serial;

      if (!camera->openCameraSerNo(id)) {
        IDSGEXXDriverLERR_ << "Failed to open uEye camera with serialNo:" << id;

        return -2;
      }

      IDSGEXXDriverLDBG_ << "Open camera with Serial:" << id;

    } else if (json.hasKey("deviceID") && json.isStringValue("deviceID")) {
      serial = json.getCStringValue("deviceID");
      uint32_t id = atoi(serial.c_str());
      if (!camera->openCameraDevId(id)) {
        IDSGEXXDriverLERR_ << "Failed to open uEye camera with deviceID" << id;

        return -4;
      }

      IDSGEXXDriverLDBG_ << "Open camera with DeviceID:" << id;

    } else if (json.hasKey("deviceID") && json.isStringValue("cameraID")) {
      serial = json.getCStringValue("cameraID");
      uint32_t id = atoi(serial.c_str());
      if (!camera->openCameraCamId(id)) {
        IDSGEXXDriverLERR_ << "Failed to open uEye camera with cameraID" << id;

        return -45;
      }

      IDSGEXXDriverLDBG_ << "Open camera with cameraID:" << id;

    } else {
      IDSGEXXDriverLDBG_ << "Looking for first camera device";
      if (!camera->openCameraSerNo(0)) {
        IDSGEXXDriverLERR_ << "Failed to open uEye Camera";

        return -4;
      }
    }
    // initialize properties.
    deinitialized = false;

    propsToCamera((chaos::common::data::CDataWrapper *)&json);
    /*  if((ret=is_InitCamera (&hCam, NULL))!=IS_SUCCESS){
        IDSGEXXDriverLERR_<<  "Cannot initalize IDS camera";
        return -1;

    }
    int32_t nPixelClockDefault = PIXEL_CLOCK;
    ret = is_PixelClock(hCam, IS_PIXELCLOCK_CMD_SET,
                        (void*)&nPixelClockDefault,
                        sizeof(nPixelClockDefault));
    if(ret!=IS_SUCCESS){
        IDSGEXXDriverLERR_<<"cannot set PIXEL CLOCK TO:"<<nPixelClockDefault;
        return -2;
    }

    int32_t colorMode = IS_CM_BGR8_PACKED;

    ret = is_SetColorMode(hCam,colorMode);

    if(ret!=IS_SUCCESS){
        IDSGEXXDriverLERR_<<"cannot set COLOR MODE TO:"<<colorMode;
        return -3;
    }
    int32_t formatID = 4;

    ret = is_ImageFormat(hCam, IMGFRMT_CMD_SET_FORMAT, &formatID,
    sizeof(formatID)); if(ret!=IS_SUCCESS){ IDSGEXXDriverLERR_<<"cannot set
    IMAGE FORMAT TO:"<<formatID; return -4;
    }
    memID = 0;
    ret = is_AllocImageMem(hCam, 2592, 1944, 16, &framebuf, &memID);
    if(ret!=IS_SUCCESS){
        IDSGEXXDriverLERR_<<"cannot set allocate framebuffer";
        return -5;
    }

    ret = is_SetImageMem(hCam, framebuf, memID);
    if(ret!=IS_SUCCESS){
        IDSGEXXDriverLERR_<<"cannot assign framebuffer";
        return -6;
    }
    // image in memory
    int32_t displayMode = IS_SET_DM_DIB;
    ret = is_SetDisplayMode (hCam, displayMode);
    if(ret!=IS_SUCCESS){
        IDSGEXXDriverLERR_<<"cannot set display";
        return -7;
    }


*/

  } catch (std::exception &e) {
    std::stringstream ss;
    ss << "Unexpected exception: \"" << e.what()
       << "\" initialitialization JSON:" << json.getJSONString();
    IDSGEXXDriverLERR_ << ss.str();
    return -110;
  } catch (...) {
    // unkonwn exception
    IDSGEXXDriverLERR_ << "an unknown exception, intialization string:"
                       << json.getJSONString();
    return -120;
  }
  initialized = true;
  createProperty("SerialNumber", (int64_t)camera->getCameraSerialNo());
  createProperty("Serial", serial);
  createProperty("Version",std::string("IDSGEXXDriver-" IDSVER) );

  createProperty("FullName", camera->getCameraName());
  camera->getFrameRate(&framerate);
  createProperty("framerate", framerate, "FRAMERATE",
                 [](AbstractDriver *thi, const std::string &name,
                    const chaos::common::data::CDataWrapper &p)
                     -> chaos::common::data::CDWUniquePtr {
                   IDSGEXXDriver *t = (IDSGEXXDriver *)thi;
                   if (t->camera->getFrameRate(&t->framerate) == 0) {
                     IDSGEXXDriverLDBG(t) << "READ FRAMERATE:" << t->framerate;
                     chaos::common::data::CDWUniquePtr ret(
                         new chaos::common::data::CDataWrapper());
                     ret->addDoubleValue(PROPERTY_VALUE_KEY, t->framerate);
                     return ret;
                   }
                   return chaos::common::data::CDWUniquePtr();
                 },
                 [](AbstractDriver *thi, const std::string &name,
                    const chaos::common::data::CDataWrapper &p)
                     -> chaos::common::data::CDWUniquePtr {
                   IDSGEXXDriver *t = (IDSGEXXDriver *)thi;
                   if (p.hasKey(PROPERTY_VALUE_KEY)) {
                     double val = p.getDoubleValue(PROPERTY_VALUE_KEY);
                     IDSGEXXDriverLDBG(t) << "WRITING FRAMERATE:" << val;

                     if (t->camera->setFrameRate(&val) == 0) {
                       t->framerate = val;
                       IDSGEXXDriverLDBG(t) << "WROTE FRAMERATE:" << val;
                     }
                     return p.clone();
                   }
                   return chaos::common::data::CDWUniquePtr();

                 });
  if ((ret = camera->getAOI(offsetx, offsety, width, height)) == IS_SUCCESS) {
    IDSGEXXDriverLDBG_ << "CURRENT OFFSET (" << offsetx << "," << offsety
                       << ") img size:" << width << "x" << height;
  }
  gain = camera->getHardwareGain();
  exposure = camera->getExposure();
  zoom = camera->getZoom();
  trgmode = IDStrgmode2trgmode(camera->getTriggerMode());
  createProperty("colorMode", framebuf_enc, FRAMEBUFFER_ENCODING_KEY,
                 [](AbstractDriver *thi, const std::string &name,
                    const chaos::common::data::CDataWrapper &p)
                     -> chaos::common::data::CDWUniquePtr {
                   IDSGEXXDriver *t = (IDSGEXXDriver *)thi;
                   uEyeColor colorMode = t->camera->getColorMode();
                   chaos::common::data::CDWUniquePtr ret(
                       new chaos::common::data::CDataWrapper());
                   t->framebuf_enc = ids2cv(colorMode);
                   ret->addStringValue(PROPERTY_VALUE_KEY, t->framebuf_enc);
                   ret->addInt32Value("raw", colorMode);
                   IDSGEXXDriverLDBG(t)
                       << "read FRAMEBUFFER_ENCODING_KEY:" << t->framebuf_enc
                       << " raw:" << colorMode;

                   return ret;

                 },
                 [](AbstractDriver *thi, const std::string &name,
                    const chaos::common::data::CDataWrapper &p)
                     -> chaos::common::data::CDWUniquePtr {
                   IDSGEXXDriver *t = (IDSGEXXDriver *)thi;
                   if (p.hasKey(PROPERTY_VALUE_KEY)) {
                     t->framebuf_enc = p.getStringValue(PROPERTY_VALUE_KEY);

                     uEyeColor val = cv2ids(t->framebuf_enc);
                     t->camera->setColorMode(val);
                     chaos::common::data::CDWUniquePtr ret = p.clone();
                     ret->addInt32Value("raw", val);
                     IDSGEXXDriverLDBG(t)
                         << "write FRAMEBUFFER_ENCODING_KEY:" << t->framebuf_enc
                         << " raw:" << val;

                     return p.clone();
                   }
                   return chaos::common::data::CDWUniquePtr();

                 });
  if ((ret = camera->getAOI(offsetx, offsety, width, height)) == IS_SUCCESS) {
    IDSGEXXDriverLDBG_ << "CURRENT OFFSET (" << offsetx << "," << offsety
                       << ") img size:" << width << "x" << height;
  }

  createProperty("width", width, 0, camera->getWidthMax(), 2, "WIDTH",
                 [](AbstractDriver *thi, const std::string &name,
                    const chaos::common::data::CDataWrapper &p)
                     -> chaos::common::data::CDWUniquePtr {
                   IDSGEXXDriver *t = (IDSGEXXDriver *)thi;

                   int ret = t->camera->getWidth();
                   if (ret >= 0) {
                     t->width = ret;

                     IDSGEXXDriverLDBG(t) << "read WIDTH:" << t->width;
                     chaos::common::data::CDWUniquePtr ret(
                         new chaos::common::data::CDataWrapper());
                     ret->addInt32Value(PROPERTY_VALUE_KEY, t->width);
                     ret->addInt32Value(PROPERTY_VALUE_MAX_KEY, t->camera->getWidthMax());

                     int w,h;
                      if(t->camera->getMinAoiSize(&w,&h)==0){
                            ret->addInt32Value(PROPERTY_VALUE_MIN_KEY, w);

                     } else {
                            ret->addInt32Value(PROPERTY_VALUE_MIN_KEY, 0);

                     }
                      if(t->camera->getMinAoiInc(&w,&h)==0){
                        ret->addInt32Value(PROPERTY_VALUE_INC_KEY, w);
                      }
                     return ret;
                   }
                   return chaos::common::data::CDWUniquePtr();

                 },
                 [](AbstractDriver *thi, const std::string &name,
                    const chaos::common::data::CDataWrapper &p)
                     -> chaos::common::data::CDWUniquePtr {
                   IDSGEXXDriver *t = (IDSGEXXDriver *)thi;
                   if (p.hasKey(PROPERTY_VALUE_KEY)) {
                     ChaosLockGuard ll(t->lock);

                     int32_t val = p.getInt32Value(PROPERTY_VALUE_KEY);
                     if (val > t->camera->getWidthMax()) {
                       val = t->camera->getWidthMax();
                     }
                     int w,h;
                      if(t->camera->getMinAoiInc(&w,&h)==0){
                        if(w>1){
                          int r=val%w;
                          if(val>w){
                            val=val-r;
                          }
                        }
                      }
                     if (t->camera->setWidth(val) == 0) {
                       IDSGEXXDriverLDBG(t) << "write WIDTH:" << val;
                       val = t->camera->getWidth();
                       if (val >= 0) {
                         t->width = val;
                       }
                       t->camera->initMemoryPool(4);
                     } else {
                       IDSGEXXDriverLERR(t)
                           << "Error writing WIDTH:" << val;
                      
                     }

                     return p.clone();
                   }
                   return chaos::common::data::CDWUniquePtr();

                 });

  createProperty("height", height, 0, camera->getHeightMax(), 2, "HEIGHT",
                 [](AbstractDriver *thi, const std::string &name,
                    const chaos::common::data::CDataWrapper &p)
                     -> chaos::common::data::CDWUniquePtr {
                   IDSGEXXDriver *t = (IDSGEXXDriver *)thi;
                   int ret = t->camera->getHeight();
                   if (ret >= 0) {

                     t->height = ret;

                     IDSGEXXDriverLDBG(t) << "read HEIGHT:" << t->height;
                     chaos::common::data::CDWUniquePtr ret(
                         new chaos::common::data::CDataWrapper());
                      int w,h;
                      
                     ret->addInt32Value(PROPERTY_VALUE_KEY, t->height);
                     ret->addInt32Value(PROPERTY_VALUE_MAX_KEY, t->camera->getHeightMax());
                     if(t->camera->getMinAoiSize(&w,&h)==0){
                            ret->addInt32Value(PROPERTY_VALUE_MIN_KEY, h);

                     } else {
                            ret->addInt32Value(PROPERTY_VALUE_MIN_KEY, 0);

                     }
                     
                      if(t->camera->getMinAoiInc(&w,&h)==0){
                        ret->addInt32Value(PROPERTY_VALUE_INC_KEY, h);
                      }
                     return ret;
                   }
                   return chaos::common::data::CDWUniquePtr();

                 },
                 [](AbstractDriver *thi, const std::string &name,
                    const chaos::common::data::CDataWrapper &p)
                     -> chaos::common::data::CDWUniquePtr {
                   IDSGEXXDriver *t = (IDSGEXXDriver *)thi;
                   if (p.hasKey(PROPERTY_VALUE_KEY)) {
                     ChaosLockGuard ll(t->lock);

                     int32_t val = p.getInt32Value(PROPERTY_VALUE_KEY);
                     if (val > t->camera->getHeightMax()) {
                       val = t->camera->getHeightMax();
                     }
                     int w,h;
                      if(t->camera->getMinAoiInc(&w,&h)==0){
                        if(w>1){
                          int r=val%h;
                          if(val>h){
                            val=val-r;
                          }
                        }
                      }
                     if (t->camera->setHeight(val) == 0) {
                       IDSGEXXDriverLDBG(t) << "write HEIGHT:" << val;
                       if ((val = t->camera->getHeight()) >= 0) {
                         t->height = val;
                       }
                       t->camera->initMemoryPool(4);
                     } else {
                       IDSGEXXDriverLERR(t)
                           << "Error writing HEIGHT:" << val;
                     }

                     return p.clone();
                   }

                   return chaos::common::data::CDWUniquePtr();

                 });

  createProperty(
      "offsetx", offsetx, 0, camera->getWidthMax(), 2, "OFFSETX",
      [](AbstractDriver *thi, const std::string &name,
         const chaos::common::data::CDataWrapper &p)
          -> chaos::common::data::CDWUniquePtr {
        IDSGEXXDriver *t = (IDSGEXXDriver *)thi;
        int ret = t->camera->getOffsetX();
        if (ret >= 0) {
          t->offsetx = ret;
          IDSGEXXDriverLDBG(t) << "read OFFSETX:" << t->offsetx;
          chaos::common::data::CDWUniquePtr ret(
              new chaos::common::data::CDataWrapper());
          ret->addInt32Value(PROPERTY_VALUE_KEY, t->offsetx);
          ret->addInt32Value(PROPERTY_VALUE_MAX_KEY, t->camera->getWidthMax() - t->offsetx);
          ret->addInt32Value(PROPERTY_VALUE_MIN_KEY, 0);
          int w,h;
          if(t->camera->getMinPosInc(&w,&h)==0){
                ret->addInt32Value(PROPERTY_VALUE_INC_KEY, w);

          } else {
                ret->addInt32Value(PROPERTY_VALUE_INC_KEY, 1);

          }
          return ret;
        }
        return chaos::common::data::CDWUniquePtr();

      },
      [](AbstractDriver *thi, const std::string &name,
         const chaos::common::data::CDataWrapper &p)
          -> chaos::common::data::CDWUniquePtr {
        IDSGEXXDriver *t = (IDSGEXXDriver *)thi;
        if (p.hasKey(PROPERTY_VALUE_KEY)) {
          int32_t val = p.getInt32Value(PROPERTY_VALUE_KEY);
          int w=0,h=0;
          if(t->camera->getMinPosInc(&w,&h)==0){
            if(val>w){
              val=val-(val%w);
            }
          }

          if (t->camera->setOffsetX(val) == 0) {
            t->offsetx = val;

            IDSGEXXDriverLDBG(t) << "write OFFSETX:" << t->offsetx;
          } else {
            IDSGEXXDriverLERR(t) << "Error writing OFFSETX:" << t->offsetx;
          }

          return p.clone();
        }
        return chaos::common::data::CDWUniquePtr();

      });

  createProperty(
      "offsety", offsety, 0, camera->getHeightMax(), 2, "OFFSETY",
      [](AbstractDriver *thi, const std::string &name,
         const chaos::common::data::CDataWrapper &p)
          -> chaos::common::data::CDWUniquePtr {
        IDSGEXXDriver *t = (IDSGEXXDriver *)thi;

        int ret = t->camera->getOffsetY();
        if (ret >= 0) {
          t->offsety = ret;
          IDSGEXXDriverLDBG(t) << "read OFFSETY:" << t->offsety;
          chaos::common::data::CDWUniquePtr ret(
              new chaos::common::data::CDataWrapper());
          ret->addInt32Value(PROPERTY_VALUE_KEY, t->offsety);
          ret->addInt32Value(PROPERTY_VALUE_MAX_KEY, t->camera->getHeightMax() - t->offsety);
          ret->addInt32Value(PROPERTY_VALUE_MIN_KEY, 0);
          int w,h;
          if(t->camera->getMinPosInc(&w,&h)==0){
                ret->addInt32Value(PROPERTY_VALUE_INC_KEY, h);

          } else {
                ret->addInt32Value(PROPERTY_VALUE_INC_KEY, 1);

          }
          return ret;
        }
        return chaos::common::data::CDWUniquePtr();

      },
      [](AbstractDriver *thi, const std::string &name,
         const chaos::common::data::CDataWrapper &p)
          -> chaos::common::data::CDWUniquePtr {
        IDSGEXXDriver *t = (IDSGEXXDriver *)thi;
        if (p.hasKey(PROPERTY_VALUE_KEY)) {
          int32_t val = p.getInt32Value(PROPERTY_VALUE_KEY);
          int w,h=0;
          if(t->camera->getMinPosInc(&w,&h)==0){
            if(val>h){
              val=val-(val%h);
            }
          }
 
          if (t->camera->setOffsetY(val) == 0) {
              t->offsety = val;

            IDSGEXXDriverLDBG(t) << "write OFFSETY:" << t->offsety;
          } else {
            IDSGEXXDriverLERR(t) << "Error writing OFFSETY:" << t->offsety;
          }

          return p.clone();
        }
        return chaos::common::data::CDWUniquePtr();

      });
  createProperty("zoom", zoom, "ZOOM",
                 [](AbstractDriver *thi, const std::string &name,
                    const chaos::common::data::CDataWrapper &p)
                     -> chaos::common::data::CDWUniquePtr {
                   IDSGEXXDriver *t = (IDSGEXXDriver *)thi;
                   t->zoom = t->camera->getZoom();
                   IDSGEXXDriverLDBG(t) << "ZOOM:" << t->zoom;
                   chaos::common::data::CDWUniquePtr ret(
                       new chaos::common::data::CDataWrapper());
                   ret->addInt32Value(PROPERTY_VALUE_KEY, t->zoom);
                   return ret;

                 },
                 [](AbstractDriver *thi, const std::string &name,
                    const chaos::common::data::CDataWrapper &p)
                     -> chaos::common::data::CDWUniquePtr {
                   IDSGEXXDriver *t = (IDSGEXXDriver *)thi;
                   if (p.hasKey(PROPERTY_VALUE_KEY)) {
                     int32_t val = p.getInt32Value(PROPERTY_VALUE_KEY);
                     t->camera->setZoom(&val);
                     t->zoom = val;

                     return p.clone();
                   }
                   return chaos::common::data::CDWUniquePtr();

                 });
  pixelclk = camera->getPixelClock();
  createProperty("pixelCLK", pixelclk, "PIXELCLK",
                 [](AbstractDriver *thi, const std::string &name,
                    const chaos::common::data::CDataWrapper &p)
                     -> chaos::common::data::CDWUniquePtr {
                   IDSGEXXDriver *t = (IDSGEXXDriver *)thi;
                   t->pixelclk = t->camera->getPixelClock();
                   int min,max,inc;
                   

                   IDSGEXXDriverLDBG(t) << "READ PIXEL CLOCK:" << t->pixelclk;
                   chaos::common::data::CDWUniquePtr ret(
                       new chaos::common::data::CDataWrapper());
                   ret->addInt32Value(PROPERTY_VALUE_KEY, t->pixelclk);
                   if(t->camera->getPixelClockRange(&min,&max,&inc)==0){
                    ret->addInt32Value(PROPERTY_VALUE_MAX_KEY, max);
                    ret->addInt32Value(PROPERTY_VALUE_MIN_KEY, min);

                   }
                   return ret;

                 },
                 [](AbstractDriver *thi, const std::string &name,
                    const chaos::common::data::CDataWrapper &p)
                     -> chaos::common::data::CDWUniquePtr {
                   IDSGEXXDriver *t = (IDSGEXXDriver *)thi;
                   if (p.hasKey(PROPERTY_VALUE_KEY)) {
                     int32_t val = p.getInt32Value(PROPERTY_VALUE_KEY);
                     int min,max,inc;
                     if(t->camera->getPixelClockRange(&min,&max,&inc)==0){
                       if(val>max){
                         val=max;
                       }
                       if(val<min){
                        val =min;
                       }
                   }
                     IDSGEXXDriverLDBG(t) << "WRITE PIXEL CLOCK:" << val;
                     t->camera->setPixelClock(&val);
                     t->pixelclk = val;
                     return p.clone();
                   }
                   return chaos::common::data::CDWUniquePtr();

                 });

  createProperty("gain", gain, "GAIN",
                 [](AbstractDriver *thi, const std::string &name,
                    const chaos::common::data::CDataWrapper &p)
                     -> chaos::common::data::CDWUniquePtr {
                   IDSGEXXDriver *t = (IDSGEXXDriver *)thi;
                   t->gain = t->camera->getHardwareGain();
                   IDSGEXXDriverLDBG(t) << "READ GAIN:" << t->gain;
                   chaos::common::data::CDWUniquePtr ret(
                       new chaos::common::data::CDataWrapper());
                   ret->addInt32Value(PROPERTY_VALUE_KEY, t->gain);
                   return ret;

                 },
                 [](AbstractDriver *thi, const std::string &name,
                    const chaos::common::data::CDataWrapper &p)
                     -> chaos::common::data::CDWUniquePtr {
                   IDSGEXXDriver *t = (IDSGEXXDriver *)thi;
                   if (p.hasKey(PROPERTY_VALUE_KEY)) {
                     int32_t val = p.getInt32Value(PROPERTY_VALUE_KEY);
                     IDSGEXXDriverLDBG(t) << "WRITE GAIN:" << val;

                     t->camera->setHardwareGain(&val);
                     t->gain =  t->camera->getHardwareGain();
                     return p.clone();
                   }

                   return chaos::common::data::CDWUniquePtr();

                 });
  createProperty("exposure", exposure, "SHUTTER",
                 [](AbstractDriver *thi, const std::string &name,
                    const chaos::common::data::CDataWrapper &p)
                     -> chaos::common::data::CDWUniquePtr {
                   IDSGEXXDriver *t = (IDSGEXXDriver *)thi;
                   t->exposure = t->camera->getExposure();
                   IDSGEXXDriverLDBG(t) << "READ SHUTTER:" << t->exposure;
                   chaos::common::data::CDWUniquePtr ret(
                       new chaos::common::data::CDataWrapper());
                   ret->addDoubleValue(PROPERTY_VALUE_KEY, t->exposure);
                   double max,min,inc;
                   if(t->camera->getExposureRange(&min,&max,&inc)==0){
                    ret->addDoubleValue(PROPERTY_VALUE_MAX_KEY, max);
                    ret->addDoubleValue(PROPERTY_VALUE_MIN_KEY, min);
                    ret->addDoubleValue(PROPERTY_VALUE_INC_KEY, inc);

                   }
                   return ret;

                 },
                 [](AbstractDriver *thi, const std::string &name,
                    const chaos::common::data::CDataWrapper &p)
                     -> chaos::common::data::CDWUniquePtr {
                   IDSGEXXDriver *t = (IDSGEXXDriver *)thi;
                   if (p.hasKey(PROPERTY_VALUE_KEY)) {
                     double val = p.getDoubleValue(PROPERTY_VALUE_KEY);
                      double max,min,inc;
                      if(t->camera->getExposureRange(&min,&max,&inc)==0){
                        if(val>max){
                          val=max;
                        }  
                        if(val<min){
                          val=min;
                        }
                      }
                     t->camera->setExposure(&val);
                     IDSGEXXDriverLDBG(t) << "WROTE SHUTTER:" << val;

                     t->exposure = val;
                     return p.clone();
                   }

                   return chaos::common::data::CDWUniquePtr();

                 });
  createProperty("camera_trigger_mode", trgmode, "TRIGGER_MODE",
                 [](AbstractDriver *thi, const std::string &name,
                    const chaos::common::data::CDataWrapper &p)
                     -> chaos::common::data::CDWUniquePtr {
                   IDSGEXXDriver *t = (IDSGEXXDriver *)thi;
                   t->trgmode =
                       IDStrgmode2trgmode(t->camera->getTriggerMode());
                   chaos::common::data::CDWUniquePtr ret(
                       new chaos::common::data::CDataWrapper());
                   ret->addInt32Value(PROPERTY_VALUE_KEY, t->trgmode);
                   return ret;

                 },
                 [](AbstractDriver *thi, const std::string &name,
                    const chaos::common::data::CDataWrapper &p)
                     -> chaos::common::data::CDWUniquePtr {
                   IDSGEXXDriver *t = (IDSGEXXDriver *)thi;
                   if (p.hasKey(PROPERTY_VALUE_KEY)) {
                     TriggerMode val =
                         trgmode2IDStrgmode(p.getInt32Value(PROPERTY_VALUE_KEY));
                     t->camera->setTriggerMode(val);
                     return p.clone();
                   }

                   return chaos::common::data::CDWUniquePtr();

                 });
  return 0;
}



/*
IDSGEXXDriver::IDSGEXXDriver():shots(0),framebuf(NULL),fn(NULL),props(NULL),tmode(CAMERA_TRIGGER_CONTINOUS),gstrategy(CAMERA_LATEST_ONLY),initialized(false){

    IDSGEXXDriverLDBG_<<  "Created Driver";
    props=new chaos::common::data::CDataWrapper();


}*/
IDSGEXXDriver::IDSGEXXDriver() : initialized(false),camera(NULL) {
  camera=new ueye::Camera();
  IDSGEXXDriverLDBG_ << "Created Driver";
}
// default descrutcor
IDSGEXXDriver::~IDSGEXXDriver() { //is_ExitCamera(hCam);
 if(camera){
   delete camera;
 }
 }
/*
#define GETINTVALUE(x,y){\
    int32_t val;\
    IDSGEXXDriverLDBG_<<"GETTING PROP \""<< # x <<"\" alias:"<<y;\
    if(getNode(# x,camera,val)==0){\
    p->addInt32Value(y,(int32_t)val);}}

#define GETINTPERCVALUE(x,y){ \
    IDSGEXXDriverLDBG_<<"GETTING PROP \""<< # x <<"\" alias:"<<y;\
    float per;\
    if(getNodeInPercentage(# x,camera,per)==0){\
    p->addInt32Value(y,(int32_t)per);}}
*/

int IDSGEXXDriver::cameraToProps(chaos::common::data::CDataWrapper *p) {

  if (p == NULL) {
    IDSGEXXDriverLERR_ << "Invalid Parameter";
    return -1;
  }
  /* if(deinitialized){
       IDSGEXXDriverLERR_ << "Camera is deinitialized";

       return 0;
   }*/
  IDSGEXXDriverLDBG_ << " synchronize all properties..";
  syncRead(); // synchronize with real values

  appendPubPropertiesTo(*p);
  /*camera->getAOI(offsetx,offsety,width,height);
   gain=camera->getHardwareGain();
  IDSGEXXDriverLDBG_<< "WIDTH:"<<width<<" HEIGHT:"<<height<<" GAIN:"<<gain;

  if(camera->getAutoGain()){
      p->addDoubleValue("GAIN",-1);
  } else {
      p->addDoubleValue("GAIN",gain);
  }
  if(camera->getFrameRate(&framerate)==0){
      IDSGEXXDriverLDBG_<< "FRAMERATE:"<<framerate;

      p->addDoubleValue("FRAMERATE",framerate);
  } else {
      p->addDoubleValue("FRAMERATE",-2);

  }
  if(camera->getAutoExposure()){
      p->addDoubleValue("SHUTTER",-1);
      IDSGEXXDriverLDBG_<< "AUTOGAIN";

  } else {
      double exp=camera->getExposure();
      p->addDoubleValue("SHUTTER",exp);
      IDSGEXXDriverLDBG_<< "SHUTTER:"<<exp;

  }
  int pixel_clock=camera->getPixelClock();
  p->addInt32Value("PIXELCLOCK",pixel_clock);
   p->addInt32Value("WIDTH",width);
  p->addInt32Value("HEIGHT",height);

  p->addInt32Value("OFFSETX",offsetx);
  p->addInt32Value("OFFSETY",offsety);





  p->addStringValue(FRAMEBUFFER_ENCODING_KEY,ids2cv(camera->getColorMode()));
  */
  return 0;
}

#define PXL2COLOR(col)                                                         \
  if (fmt == #col) {                                                           \
    color = col;                                                               \
  }

int IDSGEXXDriver::propsToCamera(chaos::common::data::CDataWrapper *p) {
  // Get the camera control object.
  int32_t ret = 0;
  if (p == NULL) {
    IDSGEXXDriverLERR_ << "Invalid Parameter";
    return -1;
  }
  /*  if(deinitialized){
        IDSGEXXDriverLERR_ << "Camera is deinitialized";
        return -2;
    }*/
  IDSGEXXDriverLDBG_ << "setting props: " << p->getCompliantJSONString();

  setProperties(*p, true);

  // Get the parameters for setting the image area of interest (Image AOI).

  // Maximize the Image AOI.
  //   chaos::common::data::CDataWrapper*p=driver->props;
  /*
      if (p->hasKey("OFFSETX")){
           int32_t val=p->getInt32Value("OFFSETX");
           ret=camera->setOffsetX(val);

          if(ret==IS_SUCCESS){
              IDSGEXXDriverLDBG_<< "setting OFFSETX:" << offsetx;
              offsetx=val;

          }else {
              ret++;
              IDSGEXXDriverLERR_<<"Setting OFFSETX FAILED";

          }
      }
      if (p->hasKey("OFFSETY")){
          int32_t val=p->getInt32Value("OFFSETY");

           ret=camera->setOffsetY(val);
          if(ret==IS_SUCCESS){
              IDSGEXXDriverLDBG_<< "setting OFFSETY " << offsety;
              offsety=val;

          }else {
              ret++;
              IDSGEXXDriverLERR_<<"Setting OFFSETY FAILED";

          }


      }
      if ( p->hasKey("WIDTH")){
          int32_t val=p->getInt32Value("WIDTH");

           ret=camera->setWidth(val);
          if(ret==IS_SUCCESS){
              IDSGEXXDriverLDBG_<< "setting WIDTH " << width;
              width=val;

          } else {
              ret++;
              IDSGEXXDriverLERR_<<"Setting WIDTH FAILED";

          }
      }
      if ( p->hasKey("HEIGHT")){
          int32_t val=p->getInt32Value("HEIGHT");

           ret=camera->setHeight(val);
          if(ret==IS_SUCCESS){
              IDSGEXXDriverLDBG_<< "setting HEIGHT " << height;
              height=val;

          }else {
              ret++;
              IDSGEXXDriverLERR_<<"Setting HEIGHT FAILED";

          }

      }
       if(p->hasKey("GAIN")){
          double gain=p->getAsRealValue("GAIN");
          if(gain<0){
              bool auto_gain=true;
              camera->setAutoGain(&auto_gain);
              IDSGEXXDriverLDBG_<< "AUTOGAIN";

          } else {
              int igain=(400*gain)/100.0;
              IDSGEXXDriverLDBG_<< "GAIN:"<<igain;

              camera->setHardwareGain(&igain);
          }

      }
       if(p->hasKey("SHUTTER")){
          double value=p->getAsRealValue("SHUTTER");
          if((value<0)){
              bool auto_exp=true;
              camera->setAutoExposure(&auto_exp);
              IDSGEXXDriverLDBG_<< "AUTO EXPOSURE";

          } else {
              IDSGEXXDriverLDBG_<< "EXPOSURE:"<<value;

              camera->setExposure(&value);
          }
      }
       if(p->hasKey("FRAMERATE")){
          double value=p->getAsRealValue("FRAMERATE");
          IDSGEXXDriverLDBG_<< "SET FRAMERATE:"<<value;

          camera->setFrameRate(&value);
      }
        if(p->hasKey("SHARPNESS")){

          if(p->getAsRealValue("SHARPNESS")<0){


          } else {


          }
      }
      if(p->hasKey("BRIGHTNESS")){

          if(p->getAsRealValue("BRIGHTNESS")<0){

          } else {


              double gain=p->getAsRealValue("BRIGHTNESS")/100.0;
              if(gain>1.0){
                  gain=1.0;
              }


          }
      }
      if(p->hasKey("CONTRAST")){

          if(p->getAsRealValue("CONTRAST")<0){

          } else {


              double gain=p->getAsRealValue("CONTRAST")/100.0;
              if(gain>1.0){
                  gain=1.0;
              }
              if(ret==IS_SUCCESS){
                  ret++;
                  IDSGEXXDriverLERR_<<"Setting CONTRAST FAILED";
              }


          }
      }
  if(p->hasKey("ZOOM")){
          int zoom=p->getAsRealValue("ZOOM");
          if(zoom>0){

              camera->setZoom(&zoom);
          }
      }
       // Pixel Clock
      if(p->hasKey("PIXELCLOCK")){
          double value=p->getAsRealValue("PIXELCLOCK");
          int ivalue=value;
          IDSGEXXDriverLDBG_<< "PIXEL CLOCK:"<<value;

          camera->setPixelClock(&ivalue);
      }
   if(p->hasKey(TRIGGER_MODE_KEY)){
          tmode=(TriggerModes)p->getInt32Value(TRIGGER_MODE_KEY);


          // Register the standard configuration event handler for enabling
  software triggering.
          // The software trigger configuration handler replaces the default
  configuration
          // as all currently registered configuration handlers are removed by
  setting the registration mode to RegistrationMode_ReplaceAll. switch(tmode){
          case (CAMERA_TRIGGER_CONTINOUS):{
              camera->setTriggerMode(TRIGGER_OFF);
              IDSGEXXDriverLDBG_<< "TRIGGER OFF";

              break;
          }
          case CAMERA_TRIGGER_SINGLE:{
              IDSGEXXDriverLDBG_<< "TRIGGER SINGLE (NA)";

              break;
          }
          case CAMERA_TRIGGER_SOFT:{
              IDSGEXXDriverLDBG_<< "TRIGGER SOFT (NA)";

              break;
          }

          case CAMERA_TRIGGER_HW_LOW:

              if(camera->setTriggerMode(TRIGGER_HI_LO)){
                  IDSGEXXDriverLDBG_<< "TRIGGER HI->LOW";

              } else {
                  IDSGEXXDriverLERR_<< "TRIGGER HI->LOW set failed";

              }
              break;
          case CAMERA_TRIGGER_HW_HI:
              IDSGEXXDriverLDBG_<< "TRIGGER LOW->HI";

              if(camera->setTriggerMode(TRIGGER_LO_HI)){
                  IDSGEXXDriverLDBG_<< "TRIGGER LOW->HI";

              } else {
                  IDSGEXXDriverLERR_<< "TRIGGER LOW->HI set failed";

              }
              break;
          case CAMERA_DISABLE_ACQUIRE:
                  IDSGEXXDriverLDBG_<< "DISABLE ACQUIRE";
                  break;

          }
      }


      if (p->hasKey(FRAMEBUFFER_ENCODING_KEY)){
          uEyeColor  color = (uEyeColor)0;

          // Set the pixel data format.
          std::string fmt=p->getStringValue(FRAMEBUFFER_ENCODING_KEY);
          IDSGEXXDriverLDBG_<< "setting Pixel Format " <<fmt;

          camera->setColorMode(cv2ids(fmt));

      }
      */

  return ret;
}

int IDSGEXXDriver::cameraInit(void *buffer, uint32_t sizeb) {
  IDSGEXXDriverLDBG_ << "Initialization";
  // simple initialization

  // For demonstration purposes only, add sample configuration event handlers to
  // print out information about camera use and image grabbing.
  return 0;
}

int IDSGEXXDriver::cameraDeinit() {
  IDSGEXXDriverLDBG_ << "deinit";
  deinitialized = true;

  return 0;
}

int IDSGEXXDriver::startGrab(uint32_t _shots, void *_framebuf,
                             cameraGrabCallBack _fn) {
  IDSGEXXDriverLDBG_ << "Start Grabbing";
  shots = _shots;
  fn = _fn;
  ChaosLockGuard ll(lock);

  if (grabbing == false) {
    camera->initMemoryPool(4);
  }
  grabbing = true;
  switch (gstrategy) {
  case CAMERA_ONE_BY_ONE:
    break;
  case CAMERA_LATEST_ONLY:
    break;
  case CAMERA_LATEST:
    break;
  case CAMERA_INCOMING:
    break;
  }
  if (shots > 0) {

  } else {
  }
  return 0;
}
int IDSGEXXDriver::waitGrab(camera_buf_t **hostbuf, uint32_t timeout_ms) {
  int32_t ret = 0;
  size_t size_ret = 0;
  const char *buf = 0;
  ChaosLockGuard ll(lock);

  if (grabbing == false) {
    return -10;
  }

  if ((ret = camera->captureImage(timeout_ms, &buf, &size_ret)) == 0) {
    //  IDSGEXXDriverLDBG_<<"Retrieved Image
    //  "<<camera->getWidth()<<"x"<<camera->getHeight()<<"
    //  ("<<offsetx<<","<<offsety<<") raw size:"<<size_ret;
    ret = size_ret;
    if (hostbuf && buf) {
      // memcpy(hostbuf,buf,size_ret);
      *hostbuf = new camera_buf_t((uint8_t *)buf, size_ret, width, height,
                                  offsetx, offsety);
      (*hostbuf)->ts = chaos::common::utility::TimingUtil::getTimeStampInMicroseconds();

    }
  } else {
    IDSGEXXDriverLERR_ << "No Image.. Timeout? ret:" << ret;
  }

  return ret;
}
int IDSGEXXDriver::waitGrab(uint32_t timeout_ms) {

  return waitGrab(NULL, timeout_ms);
}
int IDSGEXXDriver::stopGrab() {
  ChaosLockGuard ll(lock);
  grabbing = false;
  camera->destroyMemoryPool();

  // camera->StopGrabbing();
  return 0;
}

int IDSGEXXDriver::setImageProperties(int32_t width, int32_t height,
                                      int32_t opencvImageType) {
  ChaosUniquePtr<chaos::common::data::CDataWrapper> cw(
      new chaos::common::data::CDataWrapper());

  cw->addInt32Value("WIDTH", width);
  cw->addInt32Value("HEIGHT", height);

  return propsToCamera(cw.get());
}

int IDSGEXXDriver::getImageProperties(int32_t &_width, int32_t &_height,
                                      int32_t &opencvImageType) {
  ChaosUniquePtr<chaos::common::data::CDataWrapper> cw(
      new chaos::common::data::CDataWrapper());
  int ret = -1;
  cameraToProps(cw.get());
  if (cw->hasKey("WIDTH")) {
    _width = cw->getInt32Value("WIDTH");
    ret = 0;
  }

  if (cw->hasKey("HEIGHT")) {
    _height = cw->getInt32Value("HEIGHT");
    ret++;
  }

  return (ret > 0) ? 0 : ret;
}

int IDSGEXXDriver::setCameraProperty(const std::string &propname, int32_t val) {
  ChaosUniquePtr<chaos::common::data::CDataWrapper> cw(
      new chaos::common::data::CDataWrapper());
  IDSGEXXDriverLDBG_ << "Setting \"" << propname << "\"=" << (int32_t)val;

  cw->addInt32Value(propname, (int32_t)val);

  return propsToCamera(cw.get());
}
int IDSGEXXDriver::setCameraProperty(const std::string &propname, double val) {
  ChaosUniquePtr<chaos::common::data::CDataWrapper> cw(
      new chaos::common::data::CDataWrapper());
  IDSGEXXDriverLDBG_ << "Setting \"" << propname << "\"=" << val;

  cw->addDoubleValue(propname, val);

  return propsToCamera(cw.get());
}

int IDSGEXXDriver::getCameraProperty(const std::string &propname,
                                     int32_t &val) {
  ChaosUniquePtr<chaos::common::data::CDataWrapper> cw(
      new chaos::common::data::CDataWrapper());

  cameraToProps(cw.get());

  if (cw->hasKey(propname)) {
    val = cw->getInt32Value(propname);
    return 0;
  }
  return -1;
}

int IDSGEXXDriver::getCameraProperty(const std::string &propname, double &val) {
  ChaosUniquePtr<chaos::common::data::CDataWrapper> cw(
      new chaos::common::data::CDataWrapper());

  cameraToProps(cw.get());
  if (cw->hasKey(propname)) {
    val = cw->getAsRealValue(propname);
    return 0;
  }
  return -1;
}

int IDSGEXXDriver::getCameraProperties(
    chaos::common::data::CDataWrapper &proplist) {

  return cameraToProps(&proplist);
}
chaos::common::data::CDWUniquePtr
IDSGEXXDriver::setDrvProperties(chaos::common::data::CDWUniquePtr prop) {
  if (prop.get()) {
    if (prop->hasKey("WIDTH") && prop->hasKey("HEIGHT") &&
        prop->hasKey("OFFSETX") && prop->hasKey("OFFSETY")) {
      int32_t w, h, x, y;
      w = prop->getInt32Value("WIDTH");
      h = prop->getInt32Value("HEIGHT");
      x = prop->getInt32Value("OFFSETX");
      y = prop->getInt32Value("OFFSETY");
      if (w > camera->getWidthMax()) {
        w = camera->getWidthMax();
        x = 0;
      } else {
        w = w - (w % 2);
      }
      if (h > camera->getHeightMax()) {
        h = camera->getHeightMax();
        y = 0;
      } else {
        h = h - (h % 2);
      }
      // multiple of 4
      x = x - (x % 2);
      y = y - (y % 2);
      // stopGrab();
      IDSGEXXDriverLDBG_ << "Performing AOI (rounded )" << w << "x" << h << "("
                         << x << "," << y << ")";
      ChaosLockGuard ll(lock);

      if (camera->setAOI(x, y, w, h) == 0) {
        prop->removeKey("OFFSETX");
        prop->removeKey("OFFSETY");
        prop->removeKey("WIDTH");
        prop->removeKey("HEIGHT");
        syncRead();
        IDSGEXXDriverLDBG_ << " AOI OK" << width << "x" << height << "("
                           << offsetx << "," << offsety << ")";
        camera->initMemoryPool(4);
      }
      // camera->setAOI(x,y,w,h);
      //      startGrab(0,0,NULL);
    }
    setProperties(*prop.get(), true);
  }
  return prop;
}

} // namespace camera
} // namespace sensor
} // namespace driver
