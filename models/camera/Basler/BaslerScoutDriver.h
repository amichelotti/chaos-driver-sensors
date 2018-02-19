/*
 *	BaslerScoutDriver.h
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
#ifndef BASLERDRIVER_h
#define BASLERDRIVER_h

#include <chaos/cu_toolkit/driver_manager/driver/AbstractDriverPlugin.h>
#include <driver/sensors/core/CameraDriverBridge.h>
#include <chaos/common/data/DatasetDB.h>
#include <stdint.h>
namespace cu_driver = chaos::cu::driver_manager::driver;

/*
 driver definition
 */
DEFINE_CU_DRIVER_DEFINITION_PROTOTYPE(BaslerScoutDriver)
namespace Pylon{
  class CInstantCamera;

};
namespace driver {
    namespace sensor {
        namespace camera {
        class CConfigurationEvent;

#define CAM_DEFAULT_WIDTH 659
#define CAM_DEFAULT_HEIGTH 494

class BaslerScoutDriver:public ::driver::sensor::camera::CameraDriverBridge {


   chaos::common::data::CDataWrapper* props;
 protected:
 void driverInit(const char *initParameter) throw(chaos::CException);
 void driverInit(const chaos::common::data::CDataWrapper& json) throw(chaos::CException);


  void driverDeinit() throw(chaos::CException) ;
    // This smart pointer will receive the grab result data.
     Pylon::CInstantCamera* camera;
     TriggerModes tmode; //0 continous, 1 software,2 hw,3 singleshot
     GrabStrategy gstrategy;

     uint32_t shots;
     void*framebuf;

     cameraGrabCallBack fn;
public:
	BaslerScoutDriver();
	~BaslerScoutDriver();
    int setImageProperties(uint32_t width,uint32_t height,uint32_t opencvImageType);

     int getImageProperties(uint32_t& width,uint32_t& height,uint32_t& opencvImageType);


     int setCameraProperty(const std::string& propname,uint32_t val);

     int getCameraProperty(const std::string& propname,uint32_t& val);

     int getCameraProperties(std::vector<std::string >& proplist);

     int startGrab(uint32_t shots,void*framebuf=NULL,cameraGrabCallBack=NULL);

     int waitGrab(uint32_t timeout_ms);

     int stopGrab();

     int cameraInit(void *buffer,uint32_t sizeb);

     int cameraDeinit();
        
     friend class CConfigurationEvent;
};
        }}}
#endif /* defined(__ControlUnitTest__DummyDriver__) */
