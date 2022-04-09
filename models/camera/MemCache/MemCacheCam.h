/*
 *	MemCacheCam.h
 *  Software emulated beam camera
 *	!CHAOS
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
#ifndef MemCacheCam_H
#define MemCacheCam_H
#include <libmemcached/memcached.h>

#include <chaos/cu_toolkit/driver_manager/driver/AbstractDriverPlugin.h>
#include <driver/sensors/core/CameraDriverBridge.h>
#include <chaos/common/data/DatasetDB.h>
#include <stdint.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#define PIXEL_CLOCK 2
namespace cu_driver = chaos::cu::driver_manager::driver;

/*
 driver definition
 */
DEFINE_CU_DRIVER_DEFINITION_PROTOTYPE(MemCacheCam)

namespace driver {
    namespace sensor {
        namespace camera {

#define CAM_DEFAULT_WIDTH 659
#define CAM_DEFAULT_HEIGTH 494
#define CAM_MAX_SHUTTER 1000
#define CAM_MAX_GAIN 1000
#define CAM_MAX_BRIGHTNESS 1500
class MemCacheCam:public ::driver::sensor::camera::CameraDriverBridge {


 protected:

 void driverInit(const char *initParameter) throw(chaos::CException);
 void driverInit(const chaos::common::data::CDataWrapper& json) throw(chaos::CException);

  int initializeCamera(const chaos::common::data::CDataWrapper& json) ;
  void driverDeinit() throw(chaos::CException) ;
    // This smart pointer will receive the grab result data.
     int32_t memID;
     cameraGrabCallBack fn;
     int pixelEncoding;
     std::string pixelEncoding_str,key,server;
     int propsToCamera(chaos::common::data::CDataWrapper*p);
     int cameraToProps(chaos::common::data::CDataWrapper*p);
     int width,height,offsetx,offsety,original_width,original_height,serial_id,tmode;
     double framerate;
     memcached_st *mc_client;
     int bigendian;
    uint64_t last_acquisition_ts;
     ///
     double gain_raw;
     int32_t brightness_raw,shutter_raw;
     // shape parameters
     ChaosUniquePtr<chaos::common::data::CDataWrapper> shape_params;
     //
public:
	MemCacheCam();
	~MemCacheCam();
    int setImageProperties(int32_t width,int32_t height,int32_t opencvImageType);
    void applyCameraParams(cv::Mat& img);

     int getImageProperties(int32_t& width,int32_t& height,int32_t& opencvImageType);


     int setCameraProperty(const std::string& propname,int32_t val);
     int setCameraProperty(const std::string& propname,double val);

     int getCameraProperty(const std::string& propname,int32_t& val);

     int getCameraProperty(const std::string& propname,double& val);

     int getCameraProperties(chaos::common::data::CDataWrapper& proplist);

     int startGrab(uint32_t shots,void*framebuf=NULL,cameraGrabCallBack=NULL);

     int waitGrab(uint32_t timeout_ms);
     int waitGrab(camera_buf_t**buf,uint32_t timeout_ms);

     int stopGrab();

     int cameraInit(void *buffer,uint32_t sizeb);

     int cameraDeinit();
      cu_driver::MsgManagmentResultType::MsgManagmentResult execOpcode(cu_driver::DrvMsgPtr cmd){return CameraDriverBridge::execOpcode(cmd);}

      chaos::common::data::CDWUniquePtr setDrvProperties(chaos::common::data::CDWUniquePtr);  
};
        }}}
#endif /* defined(__ControlUnitTest__DummyDriver__) */
