/*
 *	AravisDriver.h
 *	!CHAOS
 *	Created by Andrea Michelotti.
 *
 *    	Copyright 2022 INFN, National Institute of Nuclear Physics
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
#include "AravisDriver.h"
#include <chaos/cu_toolkit/driver_manager/driver/AbstractDriverPlugin.h>
#include <stdlib.h>
#include <string>

namespace cu_driver = chaos::cu::driver_manager::driver;
using namespace ::driver::sensor::camera;
#define AravisDriverLAPP_ LAPP_ << "[AravisDriver] "
#define AravisDriverLDBG LDBG_ << "[AravisDriver] "
#define AravisDriverLERR LERR_ << "[AravisDriver] "

#define AravisDriverLDBG_                                                 \
  LDBG_ << "[AravisDriver (" << serial_dev << "," << friendly_name        \
        << "):" << __FUNCTION__ << "] "
#define AravisDriverLERR_                                                 \
  LERR_ << "[AravisDriver (" << serial_dev << "," << friendly_name        \
        << "):" << __PRETTY_FUNCTION__ << "] "
#define AravisDriverLERR LERR_ << "[AravisDriver] "
// GET_PLUGIN_CLASS_DEFINITION
// we need only to define the driver because we don't are makeing a plugin
OPEN_CU_DRIVER_PLUGIN_CLASS_DEFINITION(AravisDriver, 1.0.0, ::driver::sensor::camera::AravisDriver)
REGISTER_CU_DRIVER_PLUGIN_CLASS_INIT_ATTRIBUTE(::driver::sensor::camera::AravisDriver, http_address / dnsname: port)
CLOSE_CU_DRIVER_PLUGIN_CLASS_DEFINITION
OPEN_REGISTER_PLUGIN
REGISTER_PLUGIN(::driver::sensor::camera::AravisDriver)
CLOSE_REGISTER_PLUGIN

#define GETINTNODE(x, camp, pub)                                               \
  {                                                                            \
    int32_t val;                                                               \
    LDBG_ << "GETTING INT PROP \"" << #x << "\" alias:" << pub;                \
    if (getNode(#x, camp, val, pub) == 0) {                                    \
    } else {                                                                   \
      AravisDriverLERR << "cannot read ARAVIS node \"" << #x << "\"";     \
    }                                                                          \
  }
#define GETDOUBLENODE(x, camp, pub)                                            \
  {                                                                            \
    double val;                                                                \
    LDBG_ << "GETTING double PROP \"" << #x << "\" alias:" << pub;             \
    if (getNode(#x, camp, val) == 0) {                                         \
    } else {                                                                   \
      AravisDriverLERR << "cannot read ARAVIS double node \"" << #x       \
                            << "\"";                                           \
    }                                                                          \
  }

#define GETINTVALUE(x, y, LOG)                                                 \
  {                                                                            \
    int32_t val;                                                               \
    LOG << "GETTING INT PROP \"" << #x << "\" alias:" << y;                    \
    if (getNode(#x, (&cam), val) == 0) {                                       \
      p->addInt32Value(y, (int32_t)val);                                       \
    } else {                                                                   \
      AravisDriverLERR << "cannot read ARAVIS node \"" << #x << "\"";     \
    }                                                                          \
  }
#define GETDOUBLEVALUE(x, y, LOG)                                              \
  {                                                                            \
    double val;                                                                \
    LOG << "GETTING DOUBLE PROP \"" << #x << "\" alias:" << y;                 \
    if (getNode(#x, (&cam), val) == 0) {                                       \
      p->addDoubleValue(y, val);                                               \
    } else {                                                                   \
      AravisDriverLERR << "cannot read ARAVIS node \"" << #x << "\"";     \
    }                                                                          \
  }

#define GETINTPERCVALUE(x, y, LOG)                                             \
  {                                                                            \
    LOG << "GETTING INT PERCENT PROP \"" << #x << "\" alias:" << y;            \
    float per;                                                                 \
    if (getNodeInPercentage(#x, (&cam), per) == 0) {                           \
      p->addDoubleValue(y, per);                                               \
    }                                                                          \
  }

#define SETINODE(name, val, ret)                                               \
  if (setNode(name, (int32_t)val) == 0) {                                      \
    LDBG_ << "setting \"" << name << "\" = " << val;                           \
  } else {                                                                     \
    ret++;                                                                     \
    AravisDriverLERR << "ERROR setting \"" << name << "\" = " << val;     \
  }

#define STOP_GRABBING(camera)                                                  \
  if (camera == NULL) {                                                        \
    AravisDriverLERR_ << "Invalid Camera";                                \
    return -1;                                                                 \
  }                                                                            \
  if (!stopGrabbing) {                                                         \
    stopGrab();                                                                \
    restore_grab = true;                                                       \
  }

#define RESTORE_GRABBING(camera)                                               \
  if (restore_grab) {                                                          \
    startGrab(shots, NULL, fn);                                                \
  }
  

void AravisDriver::driverInit(const char *initParameter) throw(
    chaos::CException) {
  throw chaos::CException(
      -3, "You should provide a valid JSON initialization string",
      __PRETTY_FUNCTION__);
}

void AravisDriver::driverInit(
    const chaos::common::data::CDataWrapper &json) throw(chaos::CException) {
  AravisDriverLDBG_ << "Initializing ARAVIS json driver:"
                         << json.getCompliantJSONString();
  // props->appendAllElement((chaos::common::data::CDataWrapper &)json);
  parseInitCommonParams(json);
  if (initializeCamera(json) != 0) {
    throw chaos::CException(
        -1, "cannot initialize camera " + json.getCompliantJSONString(),
        __PRETTY_FUNCTION__);
  }
}

void AravisDriver::driverDeinit() throw(chaos::CException) {
  AravisDriverLAPP_ << "Deinit ARAVIS driver";
}

namespace driver {

namespace sensor {
namespace camera {

int AravisDriver::setNode(const std::string &node_name, bool val) {

  return 0;
}
int AravisDriver::setNode(const std::string &node_name, std::string val) {

  return 0;
}

int AravisDriver::setNode(const std::string &node_name, int32_t val) {

  return 0;
}

int AravisDriver::setNode(const std::string &node_name, double val) {
  
  return 0;
}

int AravisDriver::getNode(const std::string &node_name, std::string &val) {

  return 0;
}

int AravisDriver::getNode(const std::string &node_name, bool& val) {
 
  return 0;
}

int AravisDriver::getNode(const std::string &node_name, double &percent,
                               double &max, double &min, double &inc) {
 
  return 0;
}
int AravisDriver::getNode(const std::string &node_name, int32_t &percent,
                               int32_t &max, int32_t &min, int32_t &inc) {
  
  return 0;
}

} // namespace camera
} // namespace sensor
} // namespace driver

#define CREATE_VALUE_PROP(n, pub, type)                                        \
  createProperty(                                                              \
      n,                                                                       \
      [](AbstractDriver *thi, const std::string &name,                         \
         const chaos::common::data::CDataWrapper &p)                           \
          -> chaos::common::data::CDWUniquePtr {                               \
        type val, max, min, inc;                                               \
        if (((AravisDriver *)thi)->getNode(name, val, max, min, inc) ==   \
            0) {                                                               \
          chaos::common::data::CDWUniquePtr ret(                               \
              new chaos::common::data::CDataWrapper());                        \
          ret->append(PROPERTY_VALUE_KEY, val);                                           \
          ret->append(PROPERTY_VALUE_MIN_KEY, min);                                             \
          ret->append(PROPERTY_VALUE_MAX_KEY, max);                                             \
          ret->append(PROPERTY_VALUE_INC_KEY, inc);                                             \
          return ret;                                                          \
        }                                                                      \
        AravisDriverLERR << " cannot get " #type << " " << name;          \
        return chaos::common::data::CDWUniquePtr();                            \
      },                                                                       \
      [](AbstractDriver *thi, const std::string &name,                         \
         const chaos::common::data::CDataWrapper &p)                           \
          -> chaos::common::data::CDWUniquePtr {                               \
        type val = p.getValue<type>(PROPERTY_VALUE_KEY);                                  \
        if (((AravisDriver *)thi)->setNode(name, val) != 0) {             \
          AravisDriverLERR << " cannot set " #type << " " << name         \
                                << " to:" << val;                              \
          return chaos::common::data::CDWUniquePtr();                          \
        }                                                                      \
        return p.clone();                                                      \
      },                                                                       \
      pub);

#define CREATE_PROP(n, pub, typ)                                              \
  createProperty(n,                                                            \
                 [](AbstractDriver *thi, const std::string &name,              \
                    const chaos::common::data::CDataWrapper &p)                \
                     -> chaos::common::data::CDWUniquePtr {                    \
                   typ _val;                                    \
                   if (((AravisDriver *)thi)->getNode(name, _val) == 0) {  \
                     chaos::common::data::CDWUniquePtr ret(                    \
                         new chaos::common::data::CDataWrapper());             \
                     ret->append(PROPERTY_VALUE_KEY, _val);                                \
                     return ret;                                               \
                   }                                                           \
                   AravisDriverLERR << " cannot get " #typ << " "        \
                                         << name;                              \
                   return chaos::common::data::CDWUniquePtr();                 \
                 },                                                            \
                 [](AbstractDriver *thi, const std::string &name,              \
                    const chaos::common::data::CDataWrapper &p)                \
                     -> chaos::common::data::CDWUniquePtr {                    \
                   typ _val = p.getValue<typ>(PROPERTY_VALUE_KEY);                       \
                   if (((AravisDriver *)thi)->setNode(name, _val) != 0) {  \
                     AravisDriverLERR << " cannot set " #typ << " "      \
                                           << name << " to:" << _val;           \
                     return chaos::common::data::CDWUniquePtr();               \
                   }                                                           \
                   return p.clone();                                           \
                 },                                                            \
                 pub);

static int cv2ARAVIS(const std::string &fmt) {
  if (fmt == "CV_8UC1")
    return 0;//Pylon::EPixelType::PixelType_Mono8;
  if (fmt == "CV_8SC1")
    return 1;//Pylon::EPixelType::PixelType_Mono8signed;
  if (fmt == "CV_16UC1")
    return 2;//Pylon::EPixelType::PixelType_Mono16;
  if (fmt == "CV_8UC3")
    return 3;//Pylon::EPixelType::PixelType_RGB8packed;
  return  0;//Pylon::EPixelType::PixelType_Mono8;
}
static std::string ARAVIS2cv(int fmt) {
  switch (fmt) {
  case 0/*Pylon::EPixelType::PixelType_BayerBG16*/: {
    return "BAYERBG16";
  }
  case 1:/*Pylon::EPixelType::PixelType_BayerGB16:*/ {
    return "BAYERGB16";
  }
  case 2:/*Pylon::EPixelType::PixelType_YUV422packed:*/ {
    return "YUV422packed";
  }
  case 3:/*Pylon::EPixelType::PixelType_BayerBG8:*/ {
    return "BAYERBG8";
  }
  case 4:/*Pylon::EPixelType::PixelType_BayerGR16:*/ {
    return "BAYERGR16";
  }
  case 5:/*Pylon::EPixelType::PixelType_BayerRG16:*/ {
    return "BAYERRG16";
  }
  case 6:/*Pylon::EPixelType::PixelType_Mono8:*/
    return "CV_8UC1";
  case 7:/*Pylon::EPixelType::PixelType_Mono8signed:*/
    return "CV_8SC1";
  case 8:/*Pylon::EPixelType::PixelType_Mono16:*/
    return "CV_16UC1";
  case 9:/*Pylon::EPixelType::PixelType_RGB8packed:*/
    return "CV_8UC3";
  default: { return "NOT SUPPORTED"; }
  }
}


int AravisDriver::initializeCamera(
    const chaos::common::data::CDataWrapper &json) {
  int found = 0;
  GError *error = NULL;
  devicep=NULL;
  AravisDriverLDBG_ << "getting  camera informations ..";

  arv_update_device_list();
  unsigned int ndev=arv_get_n_devices ();
  AravisDriverLDBG_ << "Found "<<ndev<<" Aravis cameras";
 if (camerap&&ARV_IS_CAMERA(camerap)) {
      AravisDriverLDBG_ << "Deleting camera before";
		  g_clear_object (&camerap);

    camerap=NULL;
  }
  if (serial.empty()) {
    camerap= arv_camera_new (NULL, &error);
  } else{

    for(int cnt=0;(cnt<ndev)&&(camerap==NULL);cnt++){
      const char*name=arv_get_device_serial_nbr(cnt);
      if(name&&(std::string(name)==serial)){
        ArvDevice*device=arv_open_device(arv_get_device_id(cnt),&error);
        if(device&&(!error)){
          camerap=arv_camera_new_with_device (device,&error);
        }
        
      }
    }
    if(error){
        setLastError(std::string(error->message));

        throw chaos::CException(-1,
                                  "Cannot initialize camera " +
                                      json.getCompliantJSONString() +
                                      ": " + std::string(error->message),
                                  __PRETTY_FUNCTION__);
        return -1;
    }
    if(!ARV_IS_CAMERA(camerap)){
      throw chaos::CException(-2,
                                  "Invalid camera pointer " +
                                      json.getCompliantJSONString(),
                                  __PRETTY_FUNCTION__);
    }

    devicep=arv_camera_get_device(camerap);
    std::string model_name =arv_camera_get_model_name(camerap,NULL);
    std::string friendname =arv_camera_get_vendor_name(camerap,NULL);
    std::string fullname = arv_camera_get_device_id(camerap,NULL);
    std::string sn = arv_camera_get_device_serial_number(camerap,NULL);
    createProperty("Model", (const std::string &)model_name);
    createProperty("Friend", (const std::string &)friendname);
    createProperty("FullName", (const std::string &)fullname);
    createProperty("SerialNumber", (const std::string &)sn);
    char version[80];
    sprintf(version,"AravisDriver-%d.%d.%d",arv_get_major_version(),arv_get_minor_version(),arv_get_micro_version());
    createProperty("Version",std::string(version) );
  
    
    

     // setNode("GevSCPSPacketSize", 1500);
      
      //setNode("GevSCPD", 10000);

      setNode("AcquisitionFrameRateEnable", true);
      createProperty(
          "Width",
          [](AbstractDriver *thi, const std::string &name,
             const chaos::common::data::CDataWrapper &p)
              -> chaos::common::data::CDWUniquePtr {
            AravisDriver *t = (AravisDriver *)thi;
            int32_t max, min, inc,val,x,y,w,h;
            std::string ton;
            chaos::common::data::CDWUniquePtr ret(
                new chaos::common::data::CDataWrapper());
                if(t->camerap){
                 arv_camera_get_width_bounds(t->camerap,&min,&max,NULL);
                 inc=arv_camera_get_width_increment(t->camerap,NULL);
                 arv_camera_get_region (t->camerap,&x,&y,&w,&h,NULL);
                  LDBG_ <<" Reading \""<< name <<"\" Max:"<<max<<" Min:"<<min<<" Width:"<<w<<" Height:"<<h<<" OffsetX:"<<x<<" OffesetY:"<<y;
                  ret->addInt32Value(PROPERTY_VALUE_KEY, w);
                  ret->addInt32Value(PROPERTY_VALUE_MAX_KEY, max);
                  ret->addInt32Value(PROPERTY_VALUE_MIN_KEY, min);
                  ret->addInt32Value(PROPERTY_VALUE_INC_KEY, inc);

                }
           
            return ret;
          },
          [](AbstractDriver *thi, const std::string &name,
             const chaos::common::data::CDataWrapper &p)
              -> chaos::common::data::CDWUniquePtr {
            AravisDriver *t = (AravisDriver *)thi;


            if (p.hasKey(PROPERTY_VALUE_KEY)) {
              int32_t value = p.getInt32Value(PROPERTY_VALUE_KEY);
              LDBG_ <<" Write \""<< name <<"\""<<value ;
              if(t->camerap){
                  arv_camera_set_region (t->camerap,-1,-1,value,0,NULL);
              }
                          
              return p.clone();
            }
            LERR_ << " not value in property: " << name;

            return chaos::common::data::CDWUniquePtr();
          }, "WIDTH");

createProperty(
          "Height",
          [](AbstractDriver *thi, const std::string &name,
             const chaos::common::data::CDataWrapper &p)
              -> chaos::common::data::CDWUniquePtr {
            AravisDriver *t = (AravisDriver *)thi;
            int32_t max, min, inc,val,x,y,w,h;
            std::string ton;
            chaos::common::data::CDWUniquePtr ret(
                new chaos::common::data::CDataWrapper());
                if(t->camerap){
                 arv_camera_get_height_bounds(t->camerap,&min,&max,NULL);
                 inc=arv_camera_get_height_increment(t->camerap,NULL);
                 arv_camera_get_region (t->camerap,&x,&y,&w,&h,NULL);
                  LDBG_ <<" Reading \""<< name <<"\" Max:"<<max<<" Min:"<<min<<" Width:"<<w<<" Height:"<<h<<" OffsetX:"<<x<<" OffesetY:"<<y;
                  ret->addInt32Value(PROPERTY_VALUE_KEY, h);
                  ret->addInt32Value(PROPERTY_VALUE_MAX_KEY, max);
                  ret->addInt32Value(PROPERTY_VALUE_MIN_KEY, min);
                  ret->addInt32Value(PROPERTY_VALUE_INC_KEY, inc);

                }
           
            return ret;
          },
          [](AbstractDriver *thi, const std::string &name,
             const chaos::common::data::CDataWrapper &p)
              -> chaos::common::data::CDWUniquePtr {
            AravisDriver *t = (AravisDriver *)thi;


            if (p.hasKey(PROPERTY_VALUE_KEY)) {
              int32_t value = p.getInt32Value(PROPERTY_VALUE_KEY);
              LDBG_ <<" Write \""<< name <<"\""<<value ;
              if(t->camerap){
                  arv_camera_set_region (t->camerap,-1,-1,0,value,NULL);
              }
                          
              return p.clone();
            }
            LERR_ << " not value in property: " << name;

            return chaos::common::data::CDWUniquePtr();
          }, "HEIGHT");

createProperty(
          "OffsetX",
          [](AbstractDriver *thi, const std::string &name,
             const chaos::common::data::CDataWrapper &p)
              -> chaos::common::data::CDWUniquePtr {
            AravisDriver *t = (AravisDriver *)thi;
            int32_t max, min, inc,val,x,y,w,h;
            std::string ton;
            chaos::common::data::CDWUniquePtr ret(
                new chaos::common::data::CDataWrapper());
                if(t->camerap){
                 arv_camera_get_x_offset_bounds(t->camerap,&min,&max,NULL);
                 inc=arv_camera_get_x_offset_increment(t->camerap,NULL);
                 arv_camera_get_region (t->camerap,&x,&y,&w,&h,NULL);
                  LDBG_ <<" Reading \""<< name <<"\" Max:"<<max<<" Min:"<<min<<" Width:"<<w<<" Height:"<<h<<" OffsetX:"<<x<<" OffesetY:"<<y;
                  ret->addInt32Value(PROPERTY_VALUE_KEY, x);
                  ret->addInt32Value(PROPERTY_VALUE_MAX_KEY, max);
                  ret->addInt32Value(PROPERTY_VALUE_MIN_KEY, min);
                  ret->addInt32Value(PROPERTY_VALUE_INC_KEY, inc);

                }
           
            return ret;
          },
          [](AbstractDriver *thi, const std::string &name,
             const chaos::common::data::CDataWrapper &p)
              -> chaos::common::data::CDWUniquePtr {
            AravisDriver *t = (AravisDriver *)thi;


            if (p.hasKey(PROPERTY_VALUE_KEY)) {
              int32_t value = p.getInt32Value(PROPERTY_VALUE_KEY);
              LDBG_ <<" Write \"" <<name <<"\""<<value ;
              if(t->camerap){
                  arv_camera_set_region (t->camerap,value,-1,0,0,NULL);
              }
                          
              return p.clone();
            }
            LERR_ << " not value in property: " << name;

            return chaos::common::data::CDWUniquePtr();
          }, "OFFSETX");

createProperty(
          "OffsetY",
          [](AbstractDriver *thi, const std::string &name,
             const chaos::common::data::CDataWrapper &p)
              -> chaos::common::data::CDWUniquePtr {
            AravisDriver *t = (AravisDriver *)thi;
            int32_t max, min, inc,val,x,y,w,h;
            std::string ton;
            chaos::common::data::CDWUniquePtr ret(
                new chaos::common::data::CDataWrapper());
                if(t->camerap){
                 arv_camera_get_y_offset_bounds(t->camerap,&min,&max,NULL);
                 inc=arv_camera_get_y_offset_increment(t->camerap,NULL);
                 arv_camera_get_region (t->camerap,&x,&y,&w,&h,NULL);
                  LDBG_ <<" Reading \""<< name <<"\" Max:"<<max<<" Min:"<<min<<" Width:"<<w<<" Height:"<<h<<" OffsetX:"<<x<<" OffesetY:"<<y;
                  ret->addInt32Value(PROPERTY_VALUE_KEY, y);
                  ret->addInt32Value(PROPERTY_VALUE_MAX_KEY, max);
                  ret->addInt32Value(PROPERTY_VALUE_MIN_KEY, min);
                  ret->addInt32Value(PROPERTY_VALUE_INC_KEY, inc);

                }
           
            return ret;
          },
          [](AbstractDriver *thi, const std::string &name,
             const chaos::common::data::CDataWrapper &p)
              -> chaos::common::data::CDWUniquePtr {
            AravisDriver *t = (AravisDriver *)thi;


            if (p.hasKey(PROPERTY_VALUE_KEY)) {
              int32_t value = p.getInt32Value(PROPERTY_VALUE_KEY);
              LDBG_ <<" Write \""<< name <<"\""<<value ;
              if(t->camerap){
                  arv_camera_set_region (t->camerap,-1,value,0,0,NULL);
              }
                          
              return p.clone();
            }
            LERR_ << " not value in property: " << name;

            return chaos::common::data::CDWUniquePtr();
          }, "OFFSETY");

createProperty(
          "FrameRate",
          [](AbstractDriver *thi, const std::string &name,
             const chaos::common::data::CDataWrapper &p)
              -> chaos::common::data::CDWUniquePtr {
            AravisDriver *t = (AravisDriver *)thi;
            double max, min,val;
            chaos::common::data::CDWUniquePtr ret(
                new chaos::common::data::CDataWrapper());
                if(t->camerap){
                 arv_camera_get_frame_rate_bounds(t->camerap,&min,&max,NULL);
                 val=arv_camera_get_frame_rate(t->camerap,NULL);
                  LDBG_ <<" Reading \""<< name <<"\" Max:"<<max<<" Min:"<<min<<" VAL:"<<val;
                  ret->addDoubleValue(PROPERTY_VALUE_KEY, val);
                  ret->addDoubleValue(PROPERTY_VALUE_MAX_KEY, max);
                  ret->addDoubleValue(PROPERTY_VALUE_MIN_KEY, min);
                  ret->addDoubleValue(PROPERTY_VALUE_INC_KEY, 0);

                }
           
            return ret;
          },
          [](AbstractDriver *thi, const std::string &name,
             const chaos::common::data::CDataWrapper &p)
              -> chaos::common::data::CDWUniquePtr {
            AravisDriver *t = (AravisDriver *)thi;


            if (p.hasKey(PROPERTY_VALUE_KEY)) {
              double value = p.getDoubleValue(PROPERTY_VALUE_KEY);
              LDBG_ <<" Write \""<< name <<"\""<<value ;
              if(t->camerap){
                  arv_camera_set_frame_rate (t->camerap,value,NULL);
              }
                          
              return p.clone();
            }
            LERR_ << " not value in property: " << name;

            return chaos::common::data::CDWUniquePtr();
          }, "FRAMERATE");

createProperty(
          "Gain",
          [](AbstractDriver *thi, const std::string &name,
             const chaos::common::data::CDataWrapper &p)
              -> chaos::common::data::CDWUniquePtr {
            AravisDriver *t = (AravisDriver *)thi;
            double max, min,val;
            chaos::common::data::CDWUniquePtr ret(
                new chaos::common::data::CDataWrapper());
                if(t->camerap){
                 arv_camera_get_gain_bounds(t->camerap,&min,&max,NULL);
                 val=arv_camera_get_gain(t->camerap,NULL);
                 switch(arv_camera_get_gain_auto(t->camerap,NULL)){
                  case ARV_AUTO_ONCE:
                    val=0;
                    LDBG_ <<" Gain Auto Once";

                    break;
                  case ARV_AUTO_CONTINUOUS:
                    val=-1;
                    LDBG_ <<" Gain Continuous";

                    break;
                  default:
                  break;
                 }
                  LDBG_ <<" Reading \""<< name <<"\" Max:"<<max<<" Min:"<<min<<" VAL:"<<val;
                  ret->addDoubleValue(PROPERTY_VALUE_KEY, val);
                  ret->addDoubleValue(PROPERTY_VALUE_MAX_KEY, max);
                  ret->addDoubleValue(PROPERTY_VALUE_MIN_KEY, min);
                  ret->addDoubleValue(PROPERTY_VALUE_INC_KEY, 0);

                }
           
            return ret;
          },
          [](AbstractDriver *thi, const std::string &name,
             const chaos::common::data::CDataWrapper &p)
              -> chaos::common::data::CDWUniquePtr {
            AravisDriver *t = (AravisDriver *)thi;


            if (p.hasKey(PROPERTY_VALUE_KEY)) {
              double value = p.getDoubleValue(PROPERTY_VALUE_KEY);
              LDBG_ <<" Write \""<< name <<"\""<<value ;
              if(t->camerap){
                if(value>0){
                  arv_camera_set_gain_auto(t->camerap,ARV_AUTO_OFF,NULL);
                  arv_camera_set_gain (t->camerap,value,NULL);
                } else if(value==0){
                  LDBG_ <<" Gain Auto Once";

                  arv_camera_set_gain_auto(t->camerap,ARV_AUTO_ONCE,NULL);

                } else {
                  LDBG_ <<" Gain Auto Continuous";

                  arv_camera_set_gain_auto(t->camerap,ARV_AUTO_CONTINUOUS,NULL);

                }
              }
                          
              return p.clone();
            }
            LERR_ << " not value in property: " << name;

            return chaos::common::data::CDWUniquePtr();
          }, "GAIN");

createProperty(
          "ExposureTime",
          [](AbstractDriver *thi, const std::string &name,
             const chaos::common::data::CDataWrapper &p)
              -> chaos::common::data::CDWUniquePtr {
            AravisDriver *t = (AravisDriver *)thi;
            double max, min,val;
            chaos::common::data::CDWUniquePtr ret(
                new chaos::common::data::CDataWrapper());
                if(t->camerap){

                 arv_camera_get_exposure_time_bounds(t->camerap,&min,&max,NULL);
                 val=arv_camera_get_exposure_time(t->camerap,NULL);
                 switch(arv_camera_get_exposure_time_auto(t->camerap,NULL)){
                  case ARV_AUTO_ONCE:
                    val=0;
                    LDBG_ <<" Exposure Auto Once";

                    break;
                  case ARV_AUTO_CONTINUOUS:
                    val=-1;
                    LDBG_ <<" Exposure Continuous";

                    break;
                  default:
                  break;
                 }
                  LDBG_ <<" Reading \""<< name <<"\" Max:"<<max<<" Min:"<<min<<" VAL:"<<val;
                  ret->addDoubleValue(PROPERTY_VALUE_KEY, val);
                  ret->addDoubleValue(PROPERTY_VALUE_MAX_KEY, max);
                  ret->addDoubleValue(PROPERTY_VALUE_MIN_KEY, min);
                  ret->addDoubleValue(PROPERTY_VALUE_INC_KEY, 0);

                }
           
            return ret;
          },
          [](AbstractDriver *thi, const std::string &name,
             const chaos::common::data::CDataWrapper &p)
              -> chaos::common::data::CDWUniquePtr {
            AravisDriver *t = (AravisDriver *)thi;


            if (p.hasKey(PROPERTY_VALUE_KEY)) {
              double value = p.getDoubleValue(PROPERTY_VALUE_KEY);
              LDBG_ <<" Write \""<< name <<"\""<<value ;
              if(t->camerap){
                if(value>0){
                  arv_camera_set_exposure_time_auto(t->camerap,ARV_AUTO_OFF,NULL);
                  arv_camera_set_exposure_time (t->camerap,value,NULL);
                } else if(value==0){
                  LDBG_ <<" Exposure Auto Once";

                  arv_camera_set_gain_auto(t->camerap,ARV_AUTO_ONCE,NULL);

                } else {
                  LDBG_ <<" Exposure Auto Continuous";

                  arv_camera_set_exposure_time_auto(t->camerap,ARV_AUTO_CONTINUOUS,NULL);

                }
              }
                          
              return p.clone();
            }
            LERR_ << " not value in property: " << name;

            return chaos::common::data::CDWUniquePtr();
          }, "SHUTTER");

      createProperty("PixelFormat", "",  FRAMEBUFFER_ENCODING_KEY,                                                         
                 [](AbstractDriver *thi, const std::string &name,              
                    const chaos::common::data::CDataWrapper &p)                
                     -> chaos::common::data::CDWUniquePtr {                    
                   std::string val;
                   GError *error=NULL;
                   AravisDriver *t=(AravisDriver *)thi;
                   ArvPixelFormat px=arv_camera_get_pixel_format (t->camerap,&error);
                  if (error==NULL) {
                    chaos::common::data::CDWUniquePtr ret(new chaos::common::data::CDataWrapper());
                    std::string cv = ARAVIS2cv(px);
                    ret->append(PROPERTY_VALUE_KEY,cv);
                    const char*r=arv_camera_get_pixel_format_as_string(t->camerap,NULL);
                    if(r){
                      ret->append("raw",std::string(r));
                    }
                    return ret;                                               
                  }                                                           
                   AravisDriverLERR << " cannot get "<< name;                              
                   return chaos::common::data::CDWUniquePtr();                 
                 },                                                            
                 [](AbstractDriver *thi, const std::string &name,              
                    const chaos::common::data::CDataWrapper &p)                
                     -> chaos::common::data::CDWUniquePtr {                    
                    AravisDriver *t=(AravisDriver *)thi;
                   GError *error=NULL;

                    std::string cv =p.getStringValue(PROPERTY_VALUE_KEY);
                    ArvPixelFormat format=cv2ARAVIS(cv);
                    arv_camera_set_pixel_format (t->camerap,format,&error);
                    if(error){
                      
                      AravisDriverLERR << error->message;
                      t->setLastError(error->message);
                    }
                    return p.clone();                                                       
                 }
    );



      createProperty(
          "TriggerMode",
          [](AbstractDriver *thi, const std::string &name,
             const chaos::common::data::CDataWrapper &p)
              -> chaos::common::data::CDWUniquePtr {
            AravisDriver *t = (AravisDriver *)thi;
            int32_t max, min, inc;
            std::string ton;
            chaos::common::data::CDWUniquePtr ret(
                new chaos::common::data::CDataWrapper());

            const char* trigger_source= arv_camera_get_trigger_source(t->camerap,NULL);
            if(trigger_source){
              std::string tsource(trigger_source);
              if (tsource.find("Line") != std::string::npos) {
                const char* level=arv_camera_get_string (t->camerap, "TriggerActivation",NULL);
                if(level){
                  if(!strcmp(level,"RisingEdge")){
                    t->tmode= CAMERA_TRIGGER_HW_HI;
                    ret->addInt32Value(PROPERTY_VALUE_KEY, CAMERA_TRIGGER_HW_HI);

                  } else if(!strcmp(level,"FallingEdge")){
                    t->tmode= CAMERA_TRIGGER_HW_LOW;
                    ret->addInt32Value(PROPERTY_VALUE_KEY,CAMERA_TRIGGER_HW_LOW);

                  }
              }
            } else if (tsource.find("Software") != std::string::npos) {
                    ret->addInt32Value(PROPERTY_VALUE_KEY, CAMERA_TRIGGER_SOFT);
                    t->tmode= CAMERA_TRIGGER_SOFT;

            } else if (tsource.find("Counter") != std::string::npos) {
                ret->addInt32Value(PROPERTY_VALUE_KEY, CAMERA_TRIGGER_COUNTER);
                t->tmode= CAMERA_TRIGGER_COUNTER;

            } else if (tsource.find("Timer") != std::string::npos) {
                ret->addInt32Value(PROPERTY_VALUE_KEY, CAMERA_TRIGGER_TIMER);
                t->tmode= CAMERA_TRIGGER_TIMER;

            } else {
               ret->addInt32Value(PROPERTY_VALUE_KEY, CAMERA_TRIGGER_CONTINOUS);
              t->tmode= CAMERA_TRIGGER_CONTINOUS;
            }
            }
             
            ret->addInt32Value(PROPERTY_VALUE_MAX_KEY, CAMERA_UNDEFINED - 1);
            ret->addInt32Value(PROPERTY_VALUE_MIN_KEY, 0);
            return ret;
          },
          [](AbstractDriver *thi, const std::string &name,
             const chaos::common::data::CDataWrapper &p)
              -> chaos::common::data::CDWUniquePtr {
            AravisDriver *t = (AravisDriver *)thi;
            LDBG_ << name << " Trigger Mode WRITE" << p.getJSONString();

            if (p.hasKey(PROPERTY_VALUE_KEY)) {
              int32_t trigger_mode = p.getInt32Value(PROPERTY_VALUE_KEY);
              //
              switch (trigger_mode) {
              case (CAMERA_TRIGGER_CONTINOUS): {
                LDBG_ << name << " TRIGGER CONTINOUS";
               arv_camera_clear_triggers (t->camerap,NULL);


                break;
              }
              /*
              case CAMERA_TRIGGER_SINGLE: {
                t->setPropertyValue("TriggerMode", "On", true);
                break;
              }*/
              case CAMERA_TRIGGER_HW_LOW: {
                LDBG_ << name << " TRIGGER HW HILO";
                arv_camera_set_trigger_source(t->camerap,"Line1",NULL);
                arv_camera_set_string (t->camerap, "TriggerActivation","FallingEdge",NULL);

               // t->setPropertyValue("TriggerMode", "On", true);

               // t->setPropertyValue("TriggerActivation", "FallingEdge", true);
               // t->setPropertyValue("TriggerSource", "Line1", true);
                break;
              }
              case CAMERA_TRIGGER_SINGLE:
              case CAMERA_TRIGGER_HW_HI: {
                LDBG_ << name << " TRIGGER HW HILO";
                arv_camera_set_trigger_source(t->camerap,"Line1",NULL);
                arv_camera_set_string (t->camerap, "TriggerActivation","RisingEdge",NULL);
               

                break;
              }
              case CAMERA_TRIGGER_SOFT: {
                LDBG_ << name << " TRIGGER SOFT";
                arv_camera_set_trigger_source(t->camerap,"Software",NULL);


                break;
              }
              }
              t->tmode=(driver::sensor::camera::TriggerModes)trigger_mode;

              return p.clone();
            }
            LERR_ << " not value in property: " << name;  


            return chaos::common::data::CDWUniquePtr();
          }, "TRIGGER_MODE");
    
    if (camerap && json.hasKey("TRIGGER_MODE")) {
        int tmode = json.getInt32Value("TRIGGER_MODE");
        setPropertyValue("TriggerMode",tmode,true);

      }
    }
  return 0;
}
/*
AravisDriver::AravisDriver():camerap(NULL),shots(0),framebuf(NULL),fn(NULL),props(NULL),tmode(CAMERA_TRIGGER_CONTINOUS),gstrategy(CAMERA_LATEST_ONLY){

    AravisDriverLDBG_<<  "Created ARAVIS Driver";
    props=new chaos::common::data::CDataWrapper();


}
*/

AravisDriver::AravisDriver():camerap(NULL) {


  AravisDriverLDBG_ << "Created ARAVIS Driver ";


  shots = 0;
  fn = NULL; 

  tmode = CAMERA_TRIGGER_CONTINOUS;
  gstrategy = CAMERA_LATEST_ONLY;

  triggerHWSource = "Line1";
  stopGrabbing = true;
  restore_grab = false;
}

// default descrutcor
AravisDriver::~AravisDriver() {
  if (camerap) {
    AravisDriverLDBG_ << "deleting camera";

		g_clear_object (&camerap);
    camerap = NULL;
  }
  
}


int AravisDriver::cameraToProps(chaos::common::data::CDataWrapper *p) {
  int32_t ivalue;
  if (p == NULL) {
    AravisDriverLERR_ << "Invalid Parameter";
    return -1;
  }
  AravisDriverLDBG_ << " synchronize all properties..";
  
  syncRead(); // synchronize with real values
  appendPubPropertiesTo(*p);

  return 0;
}

int AravisDriver::propsToCamera(chaos::common::data::CDataWrapper *p) {
  // Get the camera control object.
  int ret = 0;
  if (p == NULL) {
    AravisDriverLDBG_ << "Invalid Parameter";
    return -1;
  }
  AravisDriverLDBG_ << "setting props: " << p->getCompliantJSONString();

  // Get the parameters for setting the image area of interest (Image AOI).
  setProperties(*p, true);
  
  return ret;
}

int AravisDriver::cameraInit(void *buffer, uint32_t sizeb) {
  AravisDriverLDBG_ << "Deprecated Initialization";
  // simple initialization
  if (camerap == NULL) {
    AravisDriverLERR_ << "no camera available";
    return -1;
  }
  return 0;

}

int AravisDriver::cameraDeinit() {
  AravisDriverLDBG_ << "deinit";
  if (camerap) {
     g_clear_object (&camerap);

    camerap=NULL;
  }
  return 0;
}

int AravisDriver::startGrab(uint32_t _shots, void *_framebuf,
                                 cameraGrabCallBack _fn) {
  AravisDriverLDBG_ << "Start Grabbing";
  
  stopGrabbing = false;
  return 0;
}
int AravisDriver::waitGrab(camera_buf_t **img, uint32_t timeout_ms) {
  if (camerap == NULL) {
    AravisDriverLERR_ << "Invalid Camera ";

    return -1;
  }
  uint32_t tim;
  if (stopGrabbing) {
    AravisDriverLERR_ << "Grabbing is stopped ";
    return CAMERA_GRAB_STOP;
  }
  ArvBuffer *buffer;
  GError *error = NULL;

  buffer = arv_camera_acquisition (camerap, timeout_ms*1000, &error);
  if (ARV_IS_BUFFER (buffer)) {
    int32_t x,y,w,h;
    size_t size;
    const void*pImageBuffer=arv_buffer_get_data(buffer,&size);
    arv_buffer_get_image_region(buffer,&x,&y,&w,&h);

    ArvPixelFormat px=arv_buffer_get_image_pixel_format(buffer); 
     AravisDriverLDBG_ <<"Acquired "<<w<<" x "<<h<<" ("<<x<<","<<y<<")";
			/* Display some informations about the retrieved buffer */
			
        if (img) {
          *img = new camera_buf_t((const uint8_t*)pImageBuffer, size,
                                  w,
                                  h,
                                  x,
                                  y);
         (*img)->ts = chaos::common::utility::TimingUtil::getTimeStampInMicroseconds();
    
        }
			/* Destroy the buffer */
			g_clear_object (&buffer);
		}
    if(error){
      std::string e(error->message);
      AravisDriverLERR_ << "Error :"<<e<< " ret:"<<error->code;

      setLastError(e);
      return error->code;
    }

  return 0;
}

int AravisDriver::waitGrab(uint32_t timeout_ms) {
  return waitGrab((camera_buf_t **)NULL, timeout_ms);
}
int AravisDriver::stopGrab() {
  AravisDriverLDBG_ << "Stop  Grabbing, unblock waiting";
 
  stopGrabbing = true;
 
  return 0;
}

int AravisDriver::setImageProperties(int32_t width, int32_t height,
                                          int32_t opencvImageType) {
  // props->setValue("WIDTH", width);
  // props->setValue("HEIGHT", width);
  int ret = 0;
  if (camerap) {
    // return propsToCamera(*camerap, props.get());
    ret = (setPropertyValue("WIDTH", width).get()) ? 0 : -1;
    ret += (setPropertyValue("HEIGHT", height).get()) ? 0 : -1;
  }
  return ret;
}

int AravisDriver::getImageProperties(int32_t &width, int32_t &height,
                                          int32_t &opencvImageType) {
  ChaosUniquePtr<chaos::common::data::CDataWrapper> cw(
      new chaos::common::data::CDataWrapper());
  int ret = -1;
  cameraToProps(cw.get());
  if (cw->hasKey("WIDTH")) {
    width = cw->getInt32Value("WIDTH");
    ret = 0;
  }

  if (cw->hasKey("HEIGHT")) {
    height = cw->getInt32Value("HEIGHT");
    ret++;
  }

  return (ret > 0) ? 0 : ret;
}

int AravisDriver::setCameraProperty(const std::string &propname,
                                         int32_t val) {
  ChaosUniquePtr<chaos::common::data::CDataWrapper> cw(
      new chaos::common::data::CDataWrapper());
  AravisDriverLDBG_ << "Setting Int \"" << propname
                         << "\"=" << (int32_t)val;

  cw->addInt32Value(propname, val);
  int ret = propsToCamera(cw.get());
  return ret;
}
int AravisDriver::setCameraProperty(const std::string &propname,
                                         double val) {
  ChaosUniquePtr<chaos::common::data::CDataWrapper> cw(
      new chaos::common::data::CDataWrapper());
  AravisDriverLDBG_ << "Setting double \"" << propname << "\"=" << val;

  cw->addDoubleValue(propname, val);
  int ret = propsToCamera(cw.get());
  return ret;
}

int AravisDriver::getCameraProperty(const std::string &propname,
                                         int32_t &val) {
  ChaosUniquePtr<chaos::common::data::CDataWrapper> cw(
      new chaos::common::data::CDataWrapper());

  cameraToProps(cw.get());

  if (cw->hasKey(propname)) {
    val = cw->getInt32Value(propname);
    AravisDriverLDBG_ << "Property Int \"" << propname << "\"=" << val;

    return 0;
  }
  return -1;
}

int AravisDriver::getCameraProperty(const std::string &propname,
                                         double &val) {
  ChaosUniquePtr<chaos::common::data::CDataWrapper> cw(
      new chaos::common::data::CDataWrapper());

  cameraToProps(cw.get());

  if (cw->hasKey(propname)) {
    val = cw->getDoubleValue(propname);
    AravisDriverLDBG_ << "Property Double \"" << propname << "\"=" << val;

    return 0;
  }
  return -1;
}

int AravisDriver::getCameraProperties(
    chaos::common::data::CDataWrapper &proplist) {
  int ret = cameraToProps(&proplist);
  AravisDriverLDBG_ << "PROPERTIES:" << proplist.getCompliantJSONString()
                         << " ret:" << ret;

  return ret;
}

