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
#define IDSGEXXDriverLDBG_		LDBG_ << "[IDSGEXXDriver:"<<__PRETTY_FUNCTION__<<"]"
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
void IDSGEXXDriver::driverInit(const char *initParameter) throw(chaos::CException){
    IDSGEXXDriverLAPP_ << "Initializing  driver:"<<initParameter;
    props->reset();
    if(initializeCamera(*props)!=0){
        throw chaos::CException(-3,"cannot initialize camera ",__PRETTY_FUNCTION__);

    }
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
void IDSGEXXDriver::driverInit(const chaos::common::data::CDataWrapper& json) throw(chaos::CException){
    IDSGEXXDriverLAPP_ << "Initializing  json driver:"<<json.getCompliantJSONString();
    props->reset();
    props->appendAllElement((chaos::common::data::CDataWrapper&)json);
    if(initializeCamera(*props)!=0){
        throw chaos::CException(-1,"cannot initialize camera "+json.getCompliantJSONString(),__PRETTY_FUNCTION__);
    }

}

void IDSGEXXDriver::driverDeinit() throw(chaos::CException) {
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
    if(json.hasKey("serial")&&json.isStringValue("serial")){
        std::string serial=json.getCStringValue("serial");
        uint32_t id=atoi(serial.c_str());
        if (!camera.openCameraSerNo(id)) {
            IDSGEXXDriverLERR_<<"Failed to open uEye camera with serialNo:"<< id;

            return -2;
        }

        IDSGEXXDriverLDBG_<<  "Open camera with Serial:"<<id;

    } else if(json.hasKey("deviceID")&&json.isStringValue("deviceID")){
        std::string serial=json.getCStringValue("deviceID");
        uint32_t id=atoi(serial.c_str());
        if (!camera.openCameraDevId(id)) {
            IDSGEXXDriverLERR_<<"Failed to open uEye camera with deviceID"<< id;

            return -4;
        }

        IDSGEXXDriverLDBG_<<  "Open camera with DeviceID:"<<id;

    } else if(json.hasKey("deviceID")&&json.isStringValue("cameraID")){
        std::string serial=json.getCStringValue("cameraID");
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
    initialized=true;

    return 0;
}
/*
IDSGEXXDriver::IDSGEXXDriver():shots(0),framebuf(NULL),fn(NULL),props(NULL),tmode(CAMERA_TRIGGER_CONTINOUS),gstrategy(CAMERA_LATEST_ONLY),initialized(false){

    IDSGEXXDriverLDBG_<<  "Created Driver";
    props=new chaos::common::data::CDataWrapper();


}*/
DEFAULT_CU_DRIVER_PLUGIN_CONSTRUCTOR_WITH_NS(::driver::sensor::camera, IDSGEXXDriver),shots(0),framebuf(NULL),fn(NULL),props(NULL),tmode(CAMERA_TRIGGER_CONTINOUS),gstrategy(CAMERA_LATEST_ONLY),initialized(false) {
    IDSGEXXDriverLDBG_<<  "Created Driver";
    props=new chaos::common::data::CDataWrapper();
}
//default descrutcor
IDSGEXXDriver::~IDSGEXXDriver() {
    is_ExitCamera(hCam);
    if(props){
        delete props;
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
int IDSGEXXDriver::cameraToProps(chaos::common::data::CDataWrapper*p){

    if(p == NULL){
        p = props;
    }
    int width = camera.getWidth();
    int height = camera.getHeight();
    int gain=camera.getHardwareGain();
    IDSGEXXDriverLDBG_<< "WIDTH:"<<width<<" HEIGHT:"<<height<<" GAIN:"<<gain;

    if(camera.getAutoGain()){
        p->addDoubleValue("GAIN",-1);
    } else {
        p->addDoubleValue("GAIN",gain);
    }
    double framerate;
    if(camera.getFrameRate(&framerate)){
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
        IDSGEXXDriverLDBG_<< "GAIN:"<<exp;

    }
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
    p->addInt32Value("WIDTH",width);
    p->addInt32Value("HEIGHT",height);

    p->addInt32Value("OFFSETX",0);
    p->addInt32Value("OFFSETY",0);

   // double framerate=camera.
    /* GETINTVALUE(Width,"WIDTH");
    GETINTVALUE(Height,"HEIGHT");
    GETINTVALUE(OffsetX,"OFFSETX");
    GETINTVALUE(OffsetY,"OFFSETY");
    GETINTPERCVALUE(GainRaw,"GAIN");
    GETINTPERCVALUE(ExposureTimeRaw,"SHUTTER");
    GETINTPERCVALUE(SharpnessEnhancementRaw,"SHARPNESS");
    GETINTPERCVALUE(BslBrightnessRaw,"BRIGHTNESS");
    GETINTPERCVALUE(BslContrastRaw,"CONTRAST");
*/
    return 0;
}

#define PXL2COLOR(col) \
    if(fmt == #col) {\
    color=col;}

int IDSGEXXDriver::propsToCamera(chaos::common::data::CDataWrapper*p){
    // Get the camera control object.
    int32_t ret=0;
    int posy=0,width=0,height=0,posx=0;
    if((ret=camera.getAOI(posx,posy,width,height))==IS_SUCCESS){
        IDSGEXXDriverLDBG_<< "CURRENT OFFSET (" << posx<<","<<posy<<") img size:"<<width<<"x"<<height;

    }

    if(p == NULL){
        p = props;
    }
    if(props->hasKey("TRIGGER_MODE")){
        tmode=(int32_t)props->getInt32Value("TRIGGER_MODE");


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
        case  CAMERA_TRIGGER_HW:{
            //  camera->RegisterConfiguration( new CActionTriggerConfiguration, RegistrationMode_ReplaceAll, Cleanup_Delete);

            break;
        }
        case CAMERA_TRIGGER_HW_LOW:
            IDSGEXXDriverLDBG_<< "TRIGGER HI->LOW";

            camera.setTriggerMode(TRIGGER_HI_LO);
            break;
        case CAMERA_TRIGGER_HW_HI:
            IDSGEXXDriverLDBG_<< "TRIGGER LOW->HI";

            camera.setTriggerMode(TRIGGER_LO_HI);
            break;
        }
    }
    IDSGEXXDriverLDBG_<<"setting props: " << p->getCompliantJSONString() ;

    // Get the parameters for setting the image area of interest (Image AOI).

    // Maximize the Image AOI.
    //   chaos::common::data::CDataWrapper*p=driver->props;

    if (p->hasKey("OFFSETX")){
         posx=p->getInt32Value("OFFSETX");
         ret=camera.setAOI(posx,posy,width,height);
    
        if(ret==IS_SUCCESS){
            IDSGEXXDriverLDBG_<< "setting OFFSETX:" << posx;
        }else {
            ret++;
            IDSGEXXDriverLERR_<<"Setting OFFSETX FAILED";

        }
    }
    if (p->hasKey("OFFSETY")){
        posy=p->getInt32Value("OFFSETY");
         ret=camera.setAOI(posx,posy,width,height);
        if(ret==IS_SUCCESS){
            IDSGEXXDriverLDBG_<< "setting OFFSETY " << posy;

        }else {
            ret++;
            IDSGEXXDriverLERR_<<"Setting OFFSETY FAILED";

        }


    }
    if ( p->hasKey("WIDTH")){
         width=p->getInt32Value("WIDTH");
         ret=camera.setAOI(posx,posy,width,height);
        if(ret==IS_SUCCESS){
            IDSGEXXDriverLDBG_<< "setting WIDTH " << width;
        } else {
            ret++;
            IDSGEXXDriverLERR_<<"Setting WIDTH FAILED";

        }
    }
    if ( p->hasKey("HEIGHT")){
         height=p->getInt32Value("HEIGHT");
         ret=camera.setAOI(posx,posy,width,height);
        if(ret==IS_SUCCESS){
            IDSGEXXDriverLDBG_<< "setting HEIGHT " << height;

        }else {
            ret++;
            IDSGEXXDriverLERR_<<"Setting HEIGHT FAILED";

        }

    }
    if (p->hasKey("PIXELFMT")){
        uEyeColor  color = (uEyeColor)0;

        // Set the pixel data format.
        std::string fmt=p->getStringValue("PIXELFMT");
        PXL2COLOR(MONO8);
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

        if(color>0){
            IDSGEXXDriverLDBG_<< "setting Pixel Format " <<fmt;

            camera.setColorMode(color);
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
    if(p->hasKey("FRAMERATE")){
        double value=p->getAsRealValue("FRAMERATE");
        IDSGEXXDriverLDBG_<< "FRAME RATE:"<<value;

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

    camera.closeCamera();


    return 0;
}


int IDSGEXXDriver::startGrab(uint32_t _shots,void*_framebuf,cameraGrabCallBack _fn){
    IDSGEXXDriverLDBG_<<"Start Grabbing";
    shots=_shots;
    framebuf=_framebuf;
    fn=_fn;
    int gstrategy;
    if(props->hasKey("GRAB_STRATEGY")){
        gstrategy=props->getInt32Value("GRAB_STRATEGY");
    }
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
int IDSGEXXDriver::waitGrab(const char**hostbuf,uint32_t timeout_ms){
  int32_t ret=0;
    size_t size_ret=0;
    const char *buf=0;
     if((ret=camera.captureImage(timeout_ms,&buf,&size_ret))==0){
            IDSGEXXDriverLDBG_<<"Retrieved Image "<<camera.getWidth()<<"x"<<camera.getHeight()<<" raw size:"<<size_ret;
            ret= size_ret;
            if(hostbuf&&buf){
       // memcpy(hostbuf,buf,size_ret);
                *hostbuf=buf;
        }
    } else{
      //      IDSGEXXDriverLERR_<<"No Image..";
    }
    
   


    return ret;
}
int IDSGEXXDriver::waitGrab(uint32_t timeout_ms){
  
    return waitGrab((const char**)&framebuf,timeout_ms);
}
int IDSGEXXDriver::stopGrab(){
    //camera->StopGrabbing();
    return 0;
}

int  IDSGEXXDriver::setImageProperties(int32_t width,int32_t height,int32_t opencvImageType){
    props->addInt32Value("WIDTH",width);
    props->addInt32Value("HEIGHT",height);

    return propsToCamera(props);
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
