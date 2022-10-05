/*
 *	EpicsAreaDetector.h
 *  Driver to connect Epics Area Detectors cams
 *	!CHAOS
 *	Created by Andrea Michelotti
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

#include <chaos/cu_toolkit/driver_manager/driver/AbstractDriverPlugin.h>
#include "../CameraDriverBridge.h"
#ifndef AREADETECTOR_DRIVER_h
#define AREADETECTOR_DRIVER_h

namespace cu_driver = chaos::cu::driver_manager::driver;
namespace driver {
namespace epics {
namespace common {
class EpicsGenericDriver;
}
}  // namespace epics
}  // namespace driver
/*
 driver definition
 */
DEFINE_CU_DRIVER_DEFINITION_PROTOTYPE(EpicsAreaDetector)

namespace driver
{
  namespace sensor
  {
    namespace camera
    {

      class EpicsAreaDetector : public ::driver::sensor::camera::CameraDriverBridge
      {

      protected:
        std::string serial_dev, friendly_name;
        void driverInit(const char *initParameter) throw(chaos::CException);
        void driverInit(const chaos::common::data::CDataWrapper &json) throw(chaos::CException);

        int initializeCamera(const chaos::common::data::CDataWrapper &json);
        void driverDeinit();
        // This smart pointer will receive the grab result data.
        int propsToCamera(chaos::common::data::CDataWrapper *p);
        int cameraToProps(chaos::common::data::CDataWrapper *p);
        void createProperties();

        int istrategy;

      public:
        int32_t sizex,sizey,offx,offy,size_image,dataype,colormode,cam_sizex,cam_sizey;

        int setNode(const std::string &node_name, bool val);
        int setNode(const std::string &node_name, std::string val);
        int setNode(const std::string &node_name, int32_t val);
        int setNode(const std::string &node_name, double val);
        int getNode(const std::string &node_name, std::string &val);
        int getNode(const std::string &node_name, bool &val);

        int getNode(const std::string &node_name, double &val, double &max, double &min, double &inc);
        int getNode(const std::string &node_name, int32_t &val, int32_t &max, int32_t &min, int32_t &inc);

          ::driver::epics::common::EpicsGenericDriver *devicedriver;


        EpicsAreaDetector();
        ~EpicsAreaDetector();
        int setImageProperties(int32_t width, int32_t height, int32_t opencvImageType);

        int getImageProperties(int32_t &width, int32_t &height, int32_t &opencvImageType);

        int setCameraProperty(const std::string &propname, int32_t val);
        int setCameraProperty(const std::string &propname, double val);

        int getCameraProperty(const std::string &propname, int32_t &val);

        int getCameraProperty(const std::string &propname, double &val);

        int getCameraProperties(chaos::common::data::CDataWrapper &proplist);

        int startGrab(uint32_t shots, void *framebuf = NULL, cameraGrabCallBack = NULL);

        int waitGrab(uint32_t timeout_ms);
        int waitGrab(::driver::sensor::camera::camera_buf_t **imgbuf, uint32_t timeout_ms);
        int stopGrab();

        int cameraInit(void *buffer, uint32_t sizeb);

        int cameraDeinit();
        cu_driver::MsgManagmentResultType::MsgManagmentResult execOpcode(cu_driver::DrvMsgPtr cmd) { return CameraDriverBridge::execOpcode(cmd); }

        friend class CConfigurationEvent;
        //    chaos::common::data::CDWUniquePtr setDrvProperties(chaos::common::data::CDWUniquePtr);
      };
    }
  }
}
#endif 
