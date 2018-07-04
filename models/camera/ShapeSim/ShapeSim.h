/*
 *	ShapeSim.h
 *  Software emulated beam camera
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
#ifndef ShapeSim_H
#define ShapeSim_H

#include <chaos/cu_toolkit/driver_manager/driver/AbstractDriverPlugin.h>
#include <driver/sensors/core/CameraDriverBridge.h>
#include <chaos/common/data/DatasetDB.h>
#include <stdint.h>
#define PIXEL_CLOCK 2
namespace cu_driver = chaos::cu::driver_manager::driver;

/*
 driver definition
 */
DEFINE_CU_DRIVER_DEFINITION_PROTOTYPE(ShapeSim)

namespace driver {
    namespace sensor {
        namespace camera {

#define CAM_DEFAULT_WIDTH 659
#define CAM_DEFAULT_HEIGTH 494

class ShapeSim:public ::driver::sensor::camera::CameraDriverBridge {


   chaos::common::data::CDataWrapper* props;
 protected:

 void driverInit(const char *initParameter) throw(chaos::CException);
 void driverInit(const chaos::common::data::CDataWrapper& json) throw(chaos::CException);

  int initializeCamera(const chaos::common::data::CDataWrapper& json) ;
  void driverDeinit() throw(chaos::CException) ;
    // This smart pointer will receive the grab result data.
     int32_t tmode; //0 continous, 1 software,2 hw,3 singleshot
     int32_t gstrategy;
     uint32_t shots;
     uint64_t frames;
     void*framebuf;
     int32_t memID;
     cameraGrabCallBack fn;
     bool initialized;
     int propsToCamera(chaos::common::data::CDataWrapper*p);
     int cameraToProps(chaos::common::data::CDataWrapper*p);
     int width,height,offsetx,offsety;
     double framerate;
     // shape parameters
     chaos::common::data::CDataWrapper* shape_params;
     std::string shape_type;
     int centerx,centery;
     int sizex,sizey;
     double rotangle,extangle;
     int colr,colg,colb,tickness,linetype;
     double err_centerx,err_centery;
     double err_sizex,err_sizey;
     double err_rotangle,err_extangle,err_tickness;
     //
public:
	ShapeSim();
	~ShapeSim();
    int setImageProperties(int32_t width,int32_t height,int32_t opencvImageType);

     int getImageProperties(int32_t& width,int32_t& height,int32_t& opencvImageType);


     int setCameraProperty(const std::string& propname,int32_t val);
     int setCameraProperty(const std::string& propname,double val);

     int getCameraProperty(const std::string& propname,int32_t& val);

     int getCameraProperty(const std::string& propname,double& val);

     int getCameraProperties(chaos::common::data::CDataWrapper& proplist);

     int startGrab(uint32_t shots,void*framebuf=NULL,cameraGrabCallBack=NULL);

     int waitGrab(uint32_t timeout_ms);

     int stopGrab();

     int cameraInit(void *buffer,uint32_t sizeb);

     int cameraDeinit();
        
};
        }}}
#endif /* defined(__ControlUnitTest__DummyDriver__) */
