/*
 *	IDSGEXXDriver.h
 *  Software emulated temperature driver
 *	!CHOAS
 *	Created by Andrea Michelotti
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
#ifndef IDSGEXXDriver_H
#define IDSGEXXDriver_H
#include "Camera.h"
#include <chaos/cu_toolkit/driver_manager/driver/AbstractDriverPlugin.h>
#include <driver/sensors/core/CameraDriverBridge.h>
#include <chaos/common/data/DatasetDB.h>
#include <stdint.h>
#define PIXEL_CLOCK 2
namespace cu_driver = chaos::cu::driver_manager::driver;

/*
 driver definition
 */
DEFINE_CU_DRIVER_DEFINITION_PROTOTYPE(IDSGEXXDriver)

namespace driver {
    namespace sensor {
        namespace camera {

#define CAM_DEFAULT_WIDTH 659
#define CAM_DEFAULT_HEIGTH 494

class IDSGEXXDriver: public ::driver::sensor::camera::CameraDriverBridge {


 protected:
   int get_next_image(char **mem, INT *image_id,int32_t timeout);

 void driverInit(const char *initParameter) ;
 void driverInit(const chaos::common::data::CDataWrapper& json);

  int initializeCamera(const chaos::common::data::CDataWrapper& json) ;
  void driverDeinit() ;
    // This smart pointer will receive the grab result data.
     HIDS hCam;
     ueye::Camera camera;
     int32_t memID;
     bool initialized;
     bool grabbing;
     int propsToCamera(chaos::common::data::CDataWrapper*p);
     int cameraToProps(chaos::common::data::CDataWrapper*p);
    bool deinitialized;
    boost::mutex lock;
public:
    double framerate,exposure;
    int32_t width,height,offsetx,offsety,gain,zoom,pixelclk,trgmode,old_size;
    std::string framebuf_enc;

    int32_t IDStrgmode2trgmode(ueye::TriggerMode ids);
    ueye::TriggerMode trgmode2IDStrgmode(int32_t cam);
    
	IDSGEXXDriver();
	~IDSGEXXDriver();
    int setImageProperties(int32_t width,int32_t height,int32_t opencvImageType);

     int getImageProperties(int32_t& width,int32_t& height,int32_t& opencvImageType);


     int setCameraProperty(const std::string& propname,int32_t val);
     int setCameraProperty(const std::string& propname,double val);

     int getCameraProperty(const std::string& propname,int32_t& val);

     int getCameraProperty(const std::string& propname,double& val);

     int getCameraProperties(chaos::common::data::CDataWrapper& proplist);

     int startGrab(uint32_t shots,void*framebuf=NULL,cameraGrabCallBack=NULL);

     int waitGrab(uint32_t timeout_ms);
      int waitGrab(camera_buf_t **imgbuf,uint32_t timeout_ms);
     int stopGrab();

     int cameraInit(void *buffer,uint32_t sizeb);

     int cameraDeinit();
            cu_driver::MsgManagmentResultType::MsgManagmentResult execOpcode(cu_driver::DrvMsgPtr cmd){return CameraDriverBridge::execOpcode(cmd);}
    chaos::common::data::CDWUniquePtr setDrvProperties(chaos::common::data::CDWUniquePtr);  


};
        }}}
#endif /* defined(__ControlUnitTest__DummyDriver__) */
