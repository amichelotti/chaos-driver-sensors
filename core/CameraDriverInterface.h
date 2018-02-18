/*
 *	CameraDriverInterface.h
 *	!CHAOS driver STUB
 *	Created by Andrea Michelotti
 *  Abstraction of a simple sensor driver
 *    	Copyright 2017 INFN, National Institute of Nuclear Physics
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
#ifndef __ASTRACTCAMERADRIVERINTERFACE_H__
#define __ASTRACTCAMERADRIVERINTERFACE_H__
#include <chaos/cu_toolkit/driver_manager/driver/AbstractDriverPlugin.h>
#include <driver/sensors/core/AbstractCameraDriver.h>
#include <chaos/cu_toolkit/driver_manager/driver/DriverTypes.h>
#include <chaos/cu_toolkit/driver_manager/driver/DriverAccessor.h>
namespace chaos_driver=::chaos::cu::driver_manager::driver;
namespace driver {

namespace sensor {
namespace camera{
typedef enum CameraDriverInterfaceOpcode{
    // x,y,opencv encoding
    CameraDriverInterfaceOpcode_SET_IMAGE_PROP = chaos_driver::OpcodeType::OP_USER,
    CameraDriverInterfaceOpcode_GET_IMAGE_PROP,
    CameraDriverInterfaceOpcode_SET_PROP,
    CameraDriverInterfaceOpcode_GET_PROP,
    CameraDriverInterfaceOpcode_GET_PROPERTIES,
    CameraDriverInterfaceOpcode_START_GRAB,
    CameraDriverInterfaceOpcode_WAIT_GRAB,
    CameraDriverInterfaceOpcode_STOP_GRAB,
    CameraDriverInterfaceOpcode_INIT,
    CameraDriverInterfaceOpcode_DEINIT

} CameraDriverInterfaceOpcode;


/*
 driver definition
 */

/**

    \param buf[in] is the pointer of the framebuffer
    \param blen[in] size in bytes
    \param width[in] image width
    \param heigth[in] image heigth
    \param error[in] an error occurred, or 0 if grabbing ok

*/

typedef struct {
    uint32_t arg0;
    uint32_t arg1;
    uint32_t arg2;
    int result;
    void*buffer;
    void*fn;
    char* str; //strings
    int strl; //len
} camera_params_t;

class CameraDriverInterface: public AbstractCameraDriver {


protected:
    chaos_driver::DrvMsg message;
    chaos_driver::DriverAccessor* accessor;

public:
    CameraDriverInterface(chaos_driver::DriverAccessor*_accessor):accessor(_accessor){};

    ~CameraDriverInterface();

    /**
         \brief Set Image properties, width, height, image type (opencv encoding), if the camera supports this feature the image will be resized as requested
         \param width[in] requested image width
         \param height[in] requested image height
         \param opencvImageType[in] requested pixel encoding, opencv types
         \return 0 if success
         */
    virtual int setImageProperties(uint32_t width,uint32_t height,uint32_t opencvImageType);
    /**
     \brief Get Image properties, width, height, image type (opencv encoding)
     \param width[out] requested image width
     \param height[out] requested image height
     \param opencvImageType[out] requested pixel encoding, opencv types
     \return 0 if success
     */
    virtual int getImageProperties(uint32_t& width,uint32_t& height,uint32_t& opencvImageType);
    

    /**
     \brief Set Camera property
     \param propname[in] property name
     \param val[in] value
     \return 0 if success
     */
    virtual int setCameraProperty(const std::string& propname,uint32_t val);
    /**
 \brief Get Camera property
     \param propname[in] property name
     \param val[out] value
 \return 0 if success
 */
    virtual int getCameraProperty(const std::string& propname,uint32_t& val);

    /**
     \brief Get Ima properties
         \param proplist[out] lists of camera properties
     \return 0 if success
     */
    virtual int getCameraProperties(std::vector<std::string >& proplist)=0;

    /**
     \brief Start Image Grabbing
     \param shots[in] number of shots (0 means continously) before stop grabbing
     \param framebuf[in] pointer to use to host the image (0, not use)
     \param callback[in] callback when grab a new image
     \return 0 if success
     */
    virtual int startGrab(uint32_t shots,void*framebuf=NULL,cameraGrabCallBack=NULL);

    /**
     \brief Wait Image Grabbing
     \param timeout[in] timeout in ms to wait (0 waits indefinitively)
     \return 0 if success
     */
    virtual int waitGrab(uint32_t timeout_ms);

    /**
     \brief Stop Image Grabbing

     \return 0 if success
     */
    virtual int stopGrab();
    /**
     \brief init the sensor
     \param buffer[in] initialisation opaque parameter
     \return 0 if success, error otherwise
     */
    virtual int cameraInit(void *buffer,uint32_t sizeb);
    
    /**
     \brief deinit the sensor
     \param buffer[in] initialisation opaque parameter
     \return 0 if success, error otherwise
     */
    virtual int cameraDeinit();

};
}
}
}
#endif /* defined(__ControlUnitTest__DummyDriver__) */
