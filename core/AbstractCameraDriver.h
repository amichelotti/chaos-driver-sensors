/*
 *	AbstracCameraDriver.h
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
#ifndef __ASTRACTCAMERADRIVER_H__
#define __ASTRACTCAMERADRIVER_H__
#include <chaos/cu_toolkit/driver_manager/driver/AbstractDriverPlugin.h>
#include <common/misc/data/core/Property.h>
namespace cu_driver = chaos::cu::driver_manager::driver;
#include <string>
#include <vector>
namespace driver {
namespace sensor {
namespace camera{


/**

    \param buf[in] is the pointer of the framebuffer
    \param blen[in] size in bytes
    \param width[in] image width
    \param heigth[in] image heigth
    \param error[in] an error occurred, or 0 if grabbing ok

*/
typedef int (*cameraGrabCallBack)(const void*buf,uint32_t blen,uint32_t width,uint32_t heigth) ;
//typedef boost::function<void(const void*buf,uint32_t blen,uint32_t width,uint32_t heigth, uint32_t error)> cameraGrabCallBack;
enum TriggerModes {
    CAMERA_TRIGGER_CONTINOUS,
    CAMERA_TRIGGER_SINGLE,
    CAMERA_TRIGGER_SOFT,
    CAMERA_TRIGGER_HW,
    CAMERA_TRIGGER_HW_LOW,
    CAMERA_TRIGGER_HW_HI

} ;
enum GrabStrategy {
    CAMERA_ONE_BY_ONE, // images processed in their original arrival
    CAMERA_LATEST_ONLY, // just last image
    CAMERA_LATEST, // keep last images (number is defined by NIMAGES)
    CAMERA_INCOMING // start grabbing just after the SW is waiting for
} ;
 class AbstractCameraDriver{


protected:
  common::misc::data::Property props;
public:
   AbstractCameraDriver(){}

    //! Execute a command
    /**
         \brief Set Image properties, width, height, image type (opencv encoding), if the camera supports this feature the image will be resized as requested
         \param width[in] requested image width
         \param height[in] requested image height
         \param opencvImageType[in] requested pixel encoding, opencv types
         \return 0 if success
         */
    virtual int setImageProperties(int32_t width,int32_t height,int32_t opencvImageType)=0;
    /**
     \brief Get Image properties, width, height, image type (opencv encoding)
     \param width[out] requested image width
     \param height[out] requested image height
     \param opencvImageType[out] requested pixel encoding, opencv types
     \return 0 if success
     */
    virtual int getImageProperties(int32_t& width,int32_t& height,int32_t& opencvImageType)=0;
    

    /**
     \brief Set Camera property
     \param propname[in] property name
     \param val[in] integer value
     \return 0 if success
     */
    virtual int setCameraProperty(const std::string& propname,int32_t val)=0;
   /**
    \brief Set Camera property
    \param propname[in] property name
    \param val[in] double value
    \return 0 if success
    */
    virtual int setCameraProperty(const std::string& propname,double val)=0;

   /**
 \brief Get Camera property
     \param propname[in] property name
     \param val[out] integer value
 \return 0 if success
 */
    virtual int getCameraProperty(const std::string& propname,int32_t& val)=0;
   /**
 \brief Get Camera property
     \param propname[in] property name
     \param val[out] double value
 \return 0 if success
 */
    virtual int getCameraProperty(const std::string& propname,double& val)=0;

    /**
     \brief Get Ima properties
         \param proplist[out] lists of camera properties
     \return 0 if success
     */
    virtual int getCameraProperties(chaos::common::data::CDataWrapper& proplist)=0;

    /**
     \brief Start Image Grabbing
     \param shots[in] number of shots (0 means continously) before stop grabbing
     \param framebuf[in] pointer to use to host the image (0, not use)
     \param callback[in] callback when grab a new image
     \return 0 if success
     */
    virtual int startGrab(uint32_t shots,void*framebuf=NULL,cameraGrabCallBack c=NULL)=0;

    /**
     \brief Wait Image Grabbing
     \param timeout[in] timeout in ms to wait (0 waits indefinitively)
     \return 0 if success
     */
    virtual int waitGrab(uint32_t timeout_ms)=0;


    virtual int waitGrab(const char**buf,uint32_t timeout_ms)=0;


    /**
     \brief Stop Image Grabbing

     \return 0 if success
     */
    virtual int stopGrab()=0;
    /**
     \brief init the sensor
     \param buffer[in] initialisation opaque parameter
     \return 0 if success, error otherwise
     */
    virtual int cameraInit(void *buffer,uint32_t sizeb)=0;
    
    /**
     \brief deinit the sensor
     \param buffer[in] initialisation opaque parameter
     \return 0 if success, error otherwise
     */
    virtual int cameraDeinit()=0;

};
}
}
}
#endif /* defined(__ControlUnitTest__DummyDriver__) */
