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
#include "../CameraDriverBridge.h"
#include <chaos/common/data/DatasetDB.h>
#include <stdint.h>
//#include <pylon/PylonIncludes.h>
namespace cu_driver = chaos::cu::driver_manager::driver;

/*
 driver definition
 */
DEFINE_CU_DRIVER_DEFINITION_PROTOTYPE(BaslerScoutDriver)
namespace Pylon{
  class CInstantCamera;
  class WaitObject;

};
namespace driver {
    namespace sensor {
        namespace camera {
        class CConfigurationEvent;

#define CAM_DEFAULT_WIDTH 659
#define CAM_DEFAULT_HEIGTH 494

class BaslerScoutDriver:public ::driver::sensor::camera::CameraDriverBridge {


 protected:
 std::string serial_dev,friendly_name;
 void driverInit(const char *initParameter) throw(chaos::CException);
 void driverInit(const chaos::common::data::CDataWrapper& json) throw(chaos::CException);

  int initializeCamera(const chaos::common::data::CDataWrapper& json) ;
  void driverDeinit() throw(chaos::CException) ;
    // This smart pointer will receive the grab result data.
          int propsToCamera(Pylon::CInstantCamera& camera,chaos::common::data::CDataWrapper*p);
     int cameraToProps(Pylon::CInstantCamera& camera,chaos::common::data::CDataWrapper*p);
     int changeTriggerMode(Pylon::CInstantCamera* camera,int trigger_mode);
     //int getNode(const std::string &node_name, Pylon::CInstantCamera *camera, int32_t &percent,const std::string pub="");
     //int getNode(const std::string &node_name, Pylon::CInstantCamera *camera, double &percent,const std::string pub="");

    //int getNodeInPercentage(const std::string &node_name, Pylon::CInstantCamera*camera, float &percent,const std::string& pub="");
    int istrategy;
    static std::atomic<int> pylon_initialization;

public:
    int setNode(const std::string &node_name,bool val);
    int setNode(const std::string &node_name, std::string val);
    int setNode(const std::string &node_name,int32_t val);
    int setNode(const std::string &node_name, double val);
    int getNode(const std::string &node_name, std::string&val);
    int getNode(const std::string &node_name, bool& val);

    int getNode(const std::string &node_name, double &val,double&max,double& min,double& inc);
    int getNode(const std::string &node_name, int32_t &val,int32_t&max,int32_t& min,int32_t& inc);

     Pylon::CInstantCamera* camerap;
  
	BaslerScoutDriver();
	~BaslerScoutDriver();
    int setImageProperties(int32_t width,int32_t height,int32_t opencvImageType);

     int getImageProperties(int32_t& width,int32_t& height,int32_t& opencvImageType);


     int setCameraProperty(const std::string& propname,int32_t val);
     int setCameraProperty(const std::string& propname,double val);

     int getCameraProperty(const std::string& propname,int32_t& val);

     int getCameraProperty(const std::string& propname,double& val);

     int getCameraProperties(chaos::common::data::CDataWrapper& proplist);

     int startGrab(uint32_t shots,void*framebuf=NULL,cameraGrabCallBack=NULL);

     int waitGrab(uint32_t timeout_ms);
     int waitGrab(::driver::sensor::camera::camera_buf_t**imgbuf,uint32_t timeout_ms);
     int stopGrab();

     int cameraInit(void *buffer,uint32_t sizeb);

     int cameraDeinit();
    cu_driver::MsgManagmentResultType::MsgManagmentResult execOpcode(cu_driver::DrvMsgPtr cmd){return CameraDriverBridge::execOpcode(cmd);}
    int cameraRoi(int sizex,int sizey,int x, int y);

     friend class CConfigurationEvent;
//    chaos::common::data::CDWUniquePtr setDrvProperties(chaos::common::data::CDWUniquePtr);  

};
        }}}
#endif /* defined(__ControlUnitTest__DummyDriver__) */
