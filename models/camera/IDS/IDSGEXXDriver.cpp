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
#include <stdlib.h>
#include <string>
#include <chaos/common/data/CDataWrapper.h>
#include <chaos/cu_toolkit/driver_manager/driver/AbstractDriverPlugin.h>
#include <math.h>
#include <boost/lexical_cast.hpp>

// use the pylon driver

namespace cu_driver = chaos::cu::driver_manager::driver;
using namespace ::driver::sensor::camera;
#define IDSGEXXDriverLAPP_		LAPP_ << "[IDSGEXXDriver] "
#define IDSGEXXDriverLDBG_		LDBG_ << "[IDSGEXXDriver:"<<__FUNCTION__<<"]"
#define IDSGEXXDriverLERR_		LERR_ << "[IDSGEXXDriver:"<<__PRETTY_FUNCTION__<<"]"


//GET_PLUGIN_CLASS_DEFINITION
//we need only to define the driver because we don't are makeing a plugin
OPEN_CU_DRIVER_PLUGIN_CLASS_DEFINITION(IDSGEXXDriver, 1.0.0,::driver::sensor::camera::IDSGEXXDriver)
REGISTER_CU_DRIVER_PLUGIN_CLASS_INIT_ATTRIBUTE(::driver::sensor::camera::IDSGEXXDriver, http_address/dnsname:port)
CLOSE_CU_DRIVER_PLUGIN_CLASS_DEFINITION
OPEN_REGISTER_PLUGIN
REGISTER_PLUGIN(::driver::sensor::camera::IDSGEXXDriver)
CLOSE_REGISTER_PLUGIN

using namespace ueye;
void IDSGEXXDriver::driverInit(const char *initParameter){
       throw chaos::CException(-3, "You should provide a valid JSON initialization string", __PRETTY_FUNCTION__);

}


int IDSGEXXDriver::get_next_image(char **mem, INT *image_id,int32_t timeout) {
    int ret;

    ret = is_WaitForNextImage(hCam, timeout, mem, image_id);

    switch (ret) {
    case IS_SUCCESS:
        break;
    case IS_TIMED_OUT:
        IDSGEXXDriverLERR_<<"Timeout exceeded:"<<timeout;
        return -1;
    case IS_CAPTURE_STATUS:
        IDSGEXXDriverLERR_<<"Transfer error.  Check capture status."<<timeout;

        return -2;
    default:
        return -3;
    }

    return 0;
}
void IDSGEXXDriver::driverInit(const chaos::common::data::CDataWrapper& json) {
    IDSGEXXDriverLAPP_ << "Initializing  json driver:"<<json.getCompliantJSONString();
    parseInitCommonParams(json);
    
    if(initializeCamera(json)!=0){
        throw chaos::CException(-1,"cannot initialize camera "+json.getCompliantJSONString(),__PRETTY_FUNCTION__);
    }

}

void IDSGEXXDriver::driverDeinit()  {
    IDSGEXXDriverLAPP_ << "Deinit driver";

}

namespace driver{

namespace sensor{
namespace camera{


//GET_PLUGIN_CLASS_DEFINITION
//we need to define the driver with alias version and a class that implement it
int IDSGEXXDriver::initializeCamera(const chaos::common::data::CDataWrapper& json) {
    int ret;
    const char *version;
    int major, minor, build;
    
    try{
   /* if (camera.checkVersion(major, minor, build, version)) {
        IDSGEXXDriverLDBG_<<"Loaded uEye SDK:"<<version;
    } else {
        IDSGEXXDriverLERR_<<"failed to check version!";
    }
*/
    // Make sure there is at least one camera available
    int num_cameras = camera.getNumberOfCameras();
    if (num_cameras > 0) {
        IDSGEXXDriverLDBG_<<"Found "<<num_cameras<<" cameras";

    } else {
        IDSGEXXDriverLERR_<<" NO CAMERA FOUND";
        return -1;
    }
    if(serial.size()){
        uint32_t id=atoi(serial.c_str());
        IDSGEXXDriverLDBG_<<"opening serial:"<<serial;

        if (!camera.openCameraSerNo(id)) {
            IDSGEXXDriverLERR_<<"Failed to open uEye camera with serialNo:"<< id;

            return -2;
        }

        IDSGEXXDriverLDBG_<<  "Open camera with Serial:"<<id;

    } else if(json.hasKey("deviceID")&&json.isStringValue("deviceID")){
        serial=json.getCStringValue("deviceID");
        uint32_t id=atoi(serial.c_str());
        if (!camera.openCameraDevId(id)) {
            IDSGEXXDriverLERR_<<"Failed to open uEye camera with deviceID"<< id;

            return -4;
        }

        IDSGEXXDriverLDBG_<<  "Open camera with DeviceID:"<<id;

    } else if(json.hasKey("deviceID")&&json.isStringValue("cameraID")){
        serial=json.getCStringValue("cameraID");
        uint32_t id=atoi(serial.c_str());
        if (!camera.openCameraCamId(id)) {
            IDSGEXXDriverLERR_<<"Failed to open uEye camera with cameraID"<< id;

            return -45;
        }

        IDSGEXXDriverLDBG_<<  "Open camera with cameraID:"<<id;

    } else {
        IDSGEXXDriverLDBG_<<  "Looking for first camera device";
        if (!camera.openCameraSerNo(0)) {
            IDSGEXXDriverLERR_<<"Failed to open uEye Camera";

            return -4;
        }
    }
    // initialize properties.
    deinitialized=false;

    propsToCamera((chaos::common::data::CDataWrapper*)&json);
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

    ret = is_ImageFormat(hCam, IMGFRMT_CMD_SET_FORMAT, &formatID, sizeof(formatID));
    if(ret!=IS_SUCCESS){
        IDSGEXXDriverLERR_<<"cannot set IMAGE FORMAT TO:"<<formatID;
        return -4;
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
      ss << "Unexpected exception: \"" << e.what();
      IDSGEXXDriverLERR_ << ss.str();
      return -110;
      } catch (...) {
      //unkonwn exception
      IDSGEXXDriverLERR_ << "an unknown exception ";
      return -120;

    }
    initialized=true;
    createProperty("SerialNumber", (int64_t)camera.getCameraSerialNo());
    createProperty("Serial", serial);

    createProperty("FullName", camera.getCameraName());
    createProperty("framerate", framerate,"FRAMERATE",[](AbstractDriver*thi,const std::string&name,
      const chaos::common::data::CDataWrapper &p) -> chaos::common::data::CDWUniquePtr {
        IDSGEXXDriver *t=(IDSGEXXDriver *)thi;
            if(t->camera.getFrameRate(&t->framerate)==0){
                IDSGEXXDriverLDBG_<< "FRAMERATE:"<<t->framerate;
                chaos::common::data::CDWUniquePtr ret(new chaos::common::data::CDataWrapper());
                ret->addDoubleValue("value",t->framerate);
                return ret;
            }
            return chaos::common::data::CDWUniquePtr();
      },[](AbstractDriver*thi,const std::string&name,
      const chaos::common::data::CDataWrapper &p) -> chaos::common::data::CDWUniquePtr {
        IDSGEXXDriver *t=(IDSGEXXDriver *)thi;
            if(p.hasKey("value")){
                double val=p.getDoubleValue("value");
                t->camera.setFrameRate(&val);
                return p.clone();
            }
            return chaos::common::data::CDWUniquePtr();
        
      });
      if((ret=camera.getAOI(offsetx,offsety,width,height))==IS_SUCCESS){
        IDSGEXXDriverLDBG_<< "CURRENT OFFSET (" << offsetx<<","<<offsety<<") img size:"<<width<<"x"<<height;

    }
    gain=camera.getHardwareGain();
    exposure=camera.getExposure();
    zoom=camera.getZoom();
createProperty("width", width,"WIDTH",[](AbstractDriver*thi,const std::string&name,
      const chaos::common::data::CDataWrapper &p) -> chaos::common::data::CDWUniquePtr {
        IDSGEXXDriver *t=(IDSGEXXDriver *)thi;
            t->width=t->camera.getWidth();
        
            IDSGEXXDriverLDBG_<< "WIDTH:"<<t->width;
            chaos::common::data::CDWUniquePtr ret(new chaos::common::data::CDataWrapper());
            ret->addInt32Value("value",t->width);
            return ret;
        
      },[](AbstractDriver*thi,const std::string&name,
      const chaos::common::data::CDataWrapper &p) -> chaos::common::data::CDWUniquePtr {
        IDSGEXXDriver *t=(IDSGEXXDriver *)thi;
            if(p.hasKey("value")){
                int32_t val=p.getInt32Value("value");
                if(t->camera.setWidth(val)==0){
                    return p.clone();
                }
            }
            return chaos::common::data::CDWUniquePtr();
        
      });

      createProperty("height", height,"HEIGHT",[](AbstractDriver*thi,const std::string&name,
      const chaos::common::data::CDataWrapper &p) -> chaos::common::data::CDWUniquePtr {
        IDSGEXXDriver *t=(IDSGEXXDriver *)thi;
            t->height=t->camera.getHeight();
            
                IDSGEXXDriverLDBG_<< "HEIGHT:"<<t->height;
                chaos::common::data::CDWUniquePtr ret(new chaos::common::data::CDataWrapper());
                ret->addInt32Value("value",t->height);
                return ret;
            
      },[](AbstractDriver*thi,const std::string&name,
      const chaos::common::data::CDataWrapper &p) -> chaos::common::data::CDWUniquePtr {
        IDSGEXXDriver *t=(IDSGEXXDriver *)thi;
            if(p.hasKey("value")){
                int32_t val=p.getInt32Value("value");
                if(t->camera.setHeight(val)==0){
                    return p.clone();
                }
            }
        
            return chaos::common::data::CDWUniquePtr();
        
      });

    createProperty("offsetx", offsetx,"OFFSETX",[](AbstractDriver*thi,const std::string&name,
      const chaos::common::data::CDataWrapper &p) -> chaos::common::data::CDWUniquePtr {
        IDSGEXXDriver *t=(IDSGEXXDriver *)thi;
            
            if(t->camera.getAOI(t->offsetx,t->offsety,t->width,t->height)==0){
        
                IDSGEXXDriverLDBG_<< "OFFSETX:"<<t->offsetx;
                chaos::common::data::CDWUniquePtr ret(new chaos::common::data::CDataWrapper());
                ret->addInt32Value("value",t->offsetx);
                return ret;
            } 
            return chaos::common::data::CDWUniquePtr();
        
      },[](AbstractDriver*thi,const std::string&name,
      const chaos::common::data::CDataWrapper &p) -> chaos::common::data::CDWUniquePtr {
        IDSGEXXDriver *t=(IDSGEXXDriver *)thi;
            if(p.hasKey("value")){
                int32_t val=p.getInt32Value("value");
                if(t->camera.setOffsetX(val)==0){
                    return p.clone();
                }
            }
            return chaos::common::data::CDWUniquePtr();
        
      });

createProperty("offsety", offsetx,"OFFSETY",[](AbstractDriver*thi,const std::string&name,
      const chaos::common::data::CDataWrapper &p) -> chaos::common::data::CDWUniquePtr {
        IDSGEXXDriver *t=(IDSGEXXDriver *)thi;
            
            if(t->camera.getAOI(t->offsetx,t->offsety,t->width,t->height)==0){
        
                IDSGEXXDriverLDBG_<< "OFFSETY:"<<t->offsety;
                chaos::common::data::CDWUniquePtr ret(new chaos::common::data::CDataWrapper());
                ret->addInt32Value("value",t->offsety);
                return ret;
            } 
            return chaos::common::data::CDWUniquePtr();
        
      },[](AbstractDriver*thi,const std::string&name,
      const chaos::common::data::CDataWrapper &p) -> chaos::common::data::CDWUniquePtr {
        IDSGEXXDriver *t=(IDSGEXXDriver *)thi;
            if(p.hasKey("value")){
                int32_t val=p.getInt32Value("value");
                if(t->camera.setOffsetY(val)==0){
                    return p.clone();
                }
            }
            return chaos::common::data::CDWUniquePtr();
        
      });
createProperty("zoom", zoom,"ZOOM",[](AbstractDriver*thi,const std::string&name,
      const chaos::common::data::CDataWrapper &p) -> chaos::common::data::CDWUniquePtr {
        IDSGEXXDriver *t=(IDSGEXXDriver *)thi;
            t->zoom=t->camera.getZoom();    
            IDSGEXXDriverLDBG_<< "ZOOM:"<<t->zoom;
            chaos::common::data::CDWUniquePtr ret(new chaos::common::data::CDataWrapper());
            ret->addInt32Value("value",t->zoom);
            return ret;
        
        
      },[](AbstractDriver*thi,const std::string&name,
      const chaos::common::data::CDataWrapper &p) -> chaos::common::data::CDWUniquePtr {
        IDSGEXXDriver *t=(IDSGEXXDriver *)thi;
            if(p.hasKey("value")){
                int32_t val=p.getInt32Value("value");
                t->camera.setZoom(&val);
                return p.clone();
                
            }
            return chaos::common::data::CDWUniquePtr();
        
      });
createProperty("pixelCLK", pixelclk,"PIXELCLK",[](AbstractDriver*thi,const std::string&name,
      const chaos::common::data::CDataWrapper &p) -> chaos::common::data::CDWUniquePtr {
        IDSGEXXDriver *t=(IDSGEXXDriver *)thi;
            t->pixelclk=t->camera.getPixelClock();    
            IDSGEXXDriverLDBG_<< "PIXEL CLOCK:"<<t->pixelclk;
            chaos::common::data::CDWUniquePtr ret(new chaos::common::data::CDataWrapper());
            ret->addInt32Value("value",t->pixelclk);
            return ret;
        
        
      },[](AbstractDriver*thi,const std::string&name,
      const chaos::common::data::CDataWrapper &p) -> chaos::common::data::CDWUniquePtr {
        IDSGEXXDriver *t=(IDSGEXXDriver *)thi;
            if(p.hasKey("value")){
                int32_t val=p.getInt32Value("value");
                t->camera.setPixelClock(&val);
                return p.clone();
                
            }
            return chaos::common::data::CDWUniquePtr();
        
      });

      createProperty("gain", gain,"GAIN",[](AbstractDriver*thi,const std::string&name,
      const chaos::common::data::CDataWrapper &p) -> chaos::common::data::CDWUniquePtr {
        IDSGEXXDriver *t=(IDSGEXXDriver *)thi;
            t->gain=t->camera.getHardwareGain();
            IDSGEXXDriverLDBG_<< "GAIN:"<<t->gain;
            chaos::common::data::CDWUniquePtr ret(new chaos::common::data::CDataWrapper());
            ret->addInt32Value("value",t->gain);
            return ret;
        
      },[](AbstractDriver*thi,const std::string&name,
      const chaos::common::data::CDataWrapper &p) -> chaos::common::data::CDWUniquePtr {
        IDSGEXXDriver *t=(IDSGEXXDriver *)thi;
            if(p.hasKey("value")){
                int32_t val=p.getInt32Value("value");
                t->camera.setHardwareGain(&val);
                return p.clone();
            }
        
            return chaos::common::data::CDWUniquePtr();
        
      });
 createProperty("exposure", exposure,"SHUTTER",[](AbstractDriver*thi,const std::string&name,
      const chaos::common::data::CDataWrapper &p) -> chaos::common::data::CDWUniquePtr {
        IDSGEXXDriver *t=(IDSGEXXDriver *)thi;
            t->exposure=t->camera.getExposure();
            IDSGEXXDriverLDBG_<< "SHUTTER:"<<t->exposure;
            chaos::common::data::CDWUniquePtr ret(new chaos::common::data::CDataWrapper());
            ret->addDoubleValue("value",t->exposure);
            return ret;
            
      },[](AbstractDriver*thi,const std::string&name,
      const chaos::common::data::CDataWrapper &p) -> chaos::common::data::CDWUniquePtr {
        IDSGEXXDriver *t=(IDSGEXXDriver *)thi;
            if(p.hasKey("value")){
                double val=p.getDoubleValue("value");
                t->camera.setExposure(&val);
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
DEFAULT_CU_DRIVER_PLUGIN_CONSTRUCTOR_WITH_NS(::driver::sensor::camera, IDSGEXXDriver),initialized(false) {
    IDSGEXXDriverLDBG_<<  "Created Driver";
}
//default descrutcor
IDSGEXXDriver::~IDSGEXXDriver() {
    is_ExitCamera(hCam);
    
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

static std::string ids2cv(uEyeColor fmt){
            switch(fmt){
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
static uEyeColor cv2ids(const std::string& fmt){
     if(fmt=="CV_8UC1")
        return MONO8;
    if(fmt=="CV_8SC1")
        return MONO8;        
    if(fmt=="CV_16UC1")
                return MONO16;
    if(fmt=="CV_8UC3")
                return RGB8;
    return MONO8;

}
int IDSGEXXDriver::cameraToProps(chaos::common::data::CDataWrapper*p){

   if(p==NULL){
        IDSGEXXDriverLERR_ << "Invalid Parameter";
        return -1;

    }
    if(deinitialized){
        IDSGEXXDriverLERR_ << "Camera is deinitialized";

        return 0;
    }

    syncRead();// synchronize with real values

    appendPubPropertiesTo(*p);
    /*camera.getAOI(offsetx,offsety,width,height);
     gain=camera.getHardwareGain();
    IDSGEXXDriverLDBG_<< "WIDTH:"<<width<<" HEIGHT:"<<height<<" GAIN:"<<gain;

    if(camera.getAutoGain()){
        p->addDoubleValue("GAIN",-1);
    } else {
        p->addDoubleValue("GAIN",gain);
    }
    if(camera.getFrameRate(&framerate)==0){
        IDSGEXXDriverLDBG_<< "FRAMERATE:"<<framerate;

        p->addDoubleValue("FRAMERATE",framerate);
    } else {
        p->addDoubleValue("FRAMERATE",-2);

    }
    if(camera.getAutoExposure()){
        p->addDoubleValue("SHUTTER",-1);
        IDSGEXXDriverLDBG_<< "AUTOGAIN";

    } else {
        double exp=camera.getExposure();
        p->addDoubleValue("SHUTTER",exp);
        IDSGEXXDriverLDBG_<< "SHUTTER:"<<exp;

    }
    int pixel_clock=camera.getPixelClock();
    p->addInt32Value("PIXELCLOCK",pixel_clock);
     p->addInt32Value("WIDTH",width);
    p->addInt32Value("HEIGHT",height);

    p->addInt32Value("OFFSETX",offsetx);
    p->addInt32Value("OFFSETY",offsety);

    */
    switch(camera.getTriggerMode()){
    case TRIGGER_HI_LO:
        IDSGEXXDriverLDBG_<< "TRIGGER HI->LOW";

        p->addInt32Value("TRIGGER_MODE",CAMERA_TRIGGER_HW_LOW);
        break;
    case TRIGGER_LO_HI:
        IDSGEXXDriverLDBG_<< "TRIGGER LOW->HI";
        p->addInt32Value("TRIGGER_MODE",CAMERA_TRIGGER_HW_HI);
        break;
    default:
        IDSGEXXDriverLDBG_<< "TRIGGER CONTINOUS";
        p->addInt32Value("TRIGGER_MODE",CAMERA_TRIGGER_CONTINOUS);
        break;
    }
   
    p->addStringValue(FRAMEBUFFER_ENCODING_KEY,ids2cv(camera.getColorMode()));

    return 0;
}

#define PXL2COLOR(col) \
    if(fmt == #col) {\
    color=col;}

int IDSGEXXDriver::propsToCamera(chaos::common::data::CDataWrapper*p){
    // Get the camera control object.
    int32_t ret=0;
    if(p==NULL){
        IDSGEXXDriverLERR_ << "Invalid Parameter";
        return -1;

    }
    if(deinitialized){
        IDSGEXXDriverLERR_ << "Camera is deinitialized";
        return -2;
    }
    setProperties(*p,true);
    

    
    if(p->hasKey(TRIGGER_MODE_KEY)){
        tmode=(TriggerModes)p->getInt32Value(TRIGGER_MODE_KEY);


        // Register the standard configuration event handler for enabling software triggering.
        // The software trigger configuration handler replaces the default configuration
        // as all currently registered configuration handlers are removed by setting the registration mode to RegistrationMode_ReplaceAll.
        switch(tmode){
        case (CAMERA_TRIGGER_CONTINOUS):{
            camera.setTriggerMode(TRIGGER_OFF);
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

            if(camera.setTriggerMode(TRIGGER_HI_LO)){
                IDSGEXXDriverLDBG_<< "TRIGGER HI->LOW";

            } else {
                IDSGEXXDriverLERR_<< "TRIGGER HI->LOW set failed";

            }
            break;
        case CAMERA_TRIGGER_HW_HI:
            IDSGEXXDriverLDBG_<< "TRIGGER LOW->HI";

            if(camera.setTriggerMode(TRIGGER_LO_HI)){
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
    IDSGEXXDriverLDBG_<<"setting props: " << p->getCompliantJSONString() ;

    // Get the parameters for setting the image area of interest (Image AOI).

    // Maximize the Image AOI.
    //   chaos::common::data::CDataWrapper*p=driver->props;
/*
    if (p->hasKey("OFFSETX")){
         int32_t val=p->getInt32Value("OFFSETX");
         ret=camera.setOffsetX(val);
    
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

         ret=camera.setOffsetY(val);
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

         ret=camera.setWidth(val);
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

         ret=camera.setHeight(val);
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
            camera.setAutoGain(&auto_gain);
            IDSGEXXDriverLDBG_<< "AUTOGAIN";

        } else {
            int igain=(400*gain)/100.0;
            IDSGEXXDriverLDBG_<< "GAIN:"<<igain;

            camera.setHardwareGain(&igain);
        }

    }
     if(p->hasKey("SHUTTER")){
        double value=p->getAsRealValue("SHUTTER");
        if((value<0)){
            bool auto_exp=true;
            camera.setAutoExposure(&auto_exp);
            IDSGEXXDriverLDBG_<< "AUTO EXPOSURE";

        } else {
            IDSGEXXDriverLDBG_<< "EXPOSURE:"<<value;
            
            camera.setExposure(&value);
        }
    }
     if(p->hasKey("FRAMERATE")){
        double value=p->getAsRealValue("FRAMERATE");
        IDSGEXXDriverLDBG_<< "SET FRAMERATE:"<<value;

        camera.setFrameRate(&value);
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

            camera.setZoom(&zoom);
        }
    }
     // Pixel Clock
    if(p->hasKey("PIXELCLOCK")){
        double value=p->getAsRealValue("PIXELCLOCK");
        int ivalue=value;
        IDSGEXXDriverLDBG_<< "PIXEL CLOCK:"<<value;

        camera.setPixelClock(&ivalue);
    }

  */

    if (p->hasKey(FRAMEBUFFER_ENCODING_KEY)){
        uEyeColor  color = (uEyeColor)0;

        // Set the pixel data format.
        std::string fmt=p->getStringValue(FRAMEBUFFER_ENCODING_KEY);
        /*PXL2COLOR(MONO8);
        PXL2COLOR(MONO16);
        PXL2COLOR(YUV);
        PXL2COLOR(YCbCr);
        PXL2COLOR(BGR5);
        PXL2COLOR(BGR565);
        PXL2COLOR(BGR8);
        PXL2COLOR(BGRA8);
        PXL2COLOR(BGRY8);
        PXL2COLOR(RGB8);
        PXL2COLOR(RGBA8);
        PXL2COLOR(RGBY8);
*/
        IDSGEXXDriverLDBG_<< "setting Pixel Format " <<fmt;

        camera.setColorMode(cv2ids(fmt));
        
    }


   

   
    
   
   

  

    return ret;
}

int IDSGEXXDriver::cameraInit(void *buffer,uint32_t sizeb){
    IDSGEXXDriverLDBG_<<"Initialization";
    // simple initialization




    // For demonstration purposes only, add sample configuration event handlers to print out information
    // about camera use and image grabbing.
    return 0;
}

int IDSGEXXDriver::cameraDeinit(){
    IDSGEXXDriverLDBG_<<"deinit";
    deinitialized=true;
    camera.closeCamera();


    return 0;
}


int IDSGEXXDriver::startGrab(uint32_t _shots,void*_framebuf,cameraGrabCallBack _fn){
    IDSGEXXDriverLDBG_<<"Start Grabbing";
    shots=_shots;
    fn=_fn;
    
    camera.initMemoryPool(4);
    switch(gstrategy){
    case CAMERA_ONE_BY_ONE:
        break;
    case CAMERA_LATEST_ONLY:
        break;
    case CAMERA_LATEST:
        break;
    case CAMERA_INCOMING:
        break;
    }
    if(shots>0){

    } else {
    }
    return 0;
}
int IDSGEXXDriver::waitGrab(camera_buf_t **hostbuf,uint32_t timeout_ms){
  int32_t ret=0;
    size_t size_ret=0;
    const char *buf=0;
     if((ret=camera.captureImage(timeout_ms,&buf,&size_ret))==0){
            IDSGEXXDriverLDBG_<<"Retrieved Image "<<camera.getWidth()<<"x"<<camera.getHeight()<<" raw size:"<<size_ret;
            ret= size_ret;
            if(hostbuf&&buf){
       // memcpy(hostbuf,buf,size_ret);
                *hostbuf=new camera_buf_t((uint8_t*)buf,size_ret,camera.getWidth(),camera.getHeight()) ;
        }
    } else{
      //      IDSGEXXDriverLERR_<<"No Image..";
    }
    
   


    return ret;
}
int IDSGEXXDriver::waitGrab(uint32_t timeout_ms){
  
    return waitGrab(NULL,timeout_ms);
}
int IDSGEXXDriver::stopGrab(){
    //camera->StopGrabbing();
    return 0;
}

int  IDSGEXXDriver::setImageProperties(int32_t width,int32_t height,int32_t opencvImageType){
    ChaosUniquePtr<chaos::common::data::CDataWrapper> cw(new chaos::common::data::CDataWrapper());

    cw->addInt32Value("WIDTH",width);
    cw->addInt32Value("HEIGHT",height);

    return propsToCamera(cw.get());
}

int  IDSGEXXDriver::getImageProperties(int32_t& width,int32_t& height,int32_t& opencvImageType){
    ChaosUniquePtr<chaos::common::data::CDataWrapper> cw(new chaos::common::data::CDataWrapper());
    int ret=-1;
    cameraToProps(cw.get());
    if(cw->hasKey("WIDTH")){
        width=cw->getInt32Value("WIDTH");
        ret=0;
    }

    if(cw->hasKey("HEIGHT")){
        height=cw->getInt32Value("HEIGHT");
        ret++;
    }

    return (ret>0)?0:ret;
}



int  IDSGEXXDriver::setCameraProperty(const std::string& propname,int32_t val){
    ChaosUniquePtr<chaos::common::data::CDataWrapper> cw(new chaos::common::data::CDataWrapper());
    IDSGEXXDriverLDBG_<<"Setting \"" << propname<<"\"="<<(int32_t)val ;

    cw->addInt32Value(propname,(int32_t)val);

    return propsToCamera(cw.get());

}
int  IDSGEXXDriver::setCameraProperty(const std::string& propname,double val){
    ChaosUniquePtr<chaos::common::data::CDataWrapper> cw(new chaos::common::data::CDataWrapper());
    IDSGEXXDriverLDBG_<<"Setting \"" << propname<<"\"="<<val ;

    cw->addDoubleValue(propname,val);

    return propsToCamera(cw.get());

}

int  IDSGEXXDriver::getCameraProperty(const std::string& propname,int32_t& val){
    ChaosUniquePtr<chaos::common::data::CDataWrapper> cw(new chaos::common::data::CDataWrapper());

    cameraToProps(cw.get());

    if(cw->hasKey(propname)){
        val = cw->getInt32Value(propname);
        return 0;
    }
    return -1;
}

int  IDSGEXXDriver::getCameraProperty(const std::string& propname,double& val){
    ChaosUniquePtr<chaos::common::data::CDataWrapper> cw(new chaos::common::data::CDataWrapper());

    cameraToProps(cw.get());
    if(cw->hasKey(propname)){
        val = cw->getAsRealValue(propname);
        return 0;
    }
    return -1;
}

int  IDSGEXXDriver::getCameraProperties(chaos::common::data::CDataWrapper& proplist){

    return cameraToProps(&proplist);


}
}}}
