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
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
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
#define CAM_MAX_SHUTTER 1000
#define CAM_MAX_GAIN 1000
#define CAM_MAX_BRIGHTNESS 1500
class ShapeSim:public ::driver::sensor::camera::CameraDriverBridge {


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
     double movex,movey,rot;
     double max_movex,max_movey,min_movex,min_movey;
     int32_t memID;
     cameraGrabCallBack fn;
     bool initialized;
     int pixelEncoding;
     int trigger_mode;
     int propsToCamera(chaos::common::data::CDataWrapper*p);
     int cameraToProps(chaos::common::data::CDataWrapper*p);
     int width,height,offsetx,offsety,original_width,original_height;
     float tmp_centerx,tmp_centery,tmp_sizex,tmp_sizey,tmp_rotangle,tmp_tickness;
     // beam
     double amplitude,err_amplitude,max_amplitude,inc_amplitude,tmp_amplitude;
     double rho;
     double sigmax,sigmay,err_sigmax,err_sigmay;
     ///
     double framerate,gain,brightness,shutter;
     int32_t gain_raw,brightness_raw,shutter_raw;
     // shape parameters
     ChaosUniquePtr<chaos::common::data::CDataWrapper> shape_params;
     std::string shape_type;
     int centerx,centery;
     int sizex,sizey;
     double rotangle,extangle;
     int colr,colg,colb,tickness,linetype;
     double err_centerx,err_centery;
     double err_sizex,err_sizey;
     double err_rotangle,err_extangle,err_tickness;
    int64_t last_acquisition_ts;
     //
public:
	ShapeSim();
	~ShapeSim();
    void createBeamImage(cv::Size size, cv::Mat& output,float uX,float uY, float sx, float sy, float amplitude=1.0f,float rho=0);
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
