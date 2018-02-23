/*
 *	RTCameraBase.cpp
 *	!CHAOS
 *	Created by Andrea Michelotti
 *
 *    	Copyright 2012 INFN, National Institute of Nuclear Physics
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

#include "RTCameraBase.h"
#include <driver/sensors/core/AbstractSensorDriver.h>
#include <chaos/cu_toolkit/control_manager/AttributeSharedCacheWrapper.h>
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <driver/sensors/core/CameraDriverInterface.h>
using namespace chaos;
using namespace chaos::common::data::cache;
using namespace chaos::cu::driver_manager::driver;
using namespace ::driver::sensor::camera;
using namespace chaos::cu::control_manager;
using namespace cv;
PUBLISHABLE_CONTROL_UNIT_IMPLEMENTATION(RTCameraBase)

#define RTCameraBaseLAPP_		LAPP_ << "[RTCameraBase] "
#define RTCameraBaseLDBG_		LDBG_ << "[RTCameraBase] " << __PRETTY_FUNCTION__ << " "
#define RTCameraBaseLERR_		LERR_ << "[RTCameraBase] " << __PRETTY_FUNCTION__ << "("<<__LINE__<<") "

/*
 Construct
 */
RTCameraBase::RTCameraBase(const string& _control_unit_id, const string& _control_unit_param, const ControlUnitDriverList& _control_unit_drivers):
RTAbstractControlUnit(_control_unit_id, _control_unit_param, _control_unit_drivers) {
   RTCameraBaseLAPP_ << "Created";

    driver=NULL;
}

/*
 Destructor
 */
RTCameraBase::~RTCameraBase() {
    if(driver){
        driver->cameraDeinit();
        delete driver;
        driver = NULL;
    }


}
bool  RTCameraBase::setProp(const std::string &name, int32_t value, uint32_t size){
    int ret;

    ret=driver->setCameraProperty(name,value);
    setStateVariableSeverity(StateVariableTypeAlarmDEV,"operation_not_supported", chaos::common::alarm::MultiSeverityAlarmLevelClear);

    if(ret!=0){
        setStateVariableSeverity(StateVariableTypeAlarmDEV,"operation_not_supported", chaos::common::alarm::MultiSeverityAlarmLevelWarning);

    }
    return (ret==0);
}

//!Return the definition of the control unit
/*!
The api that can be called withi this method are listed into
"Control Unit Definition Public API" module into html documentation
(chaosframework/Documentation/html/group___control___unit___definition___api.html)
*/
void RTCameraBase::unitDefineActionAndDataset() throw(chaos::CException) {
    addAttributeToDataSet("WIDTH","X image size",chaos::DataType::TYPE_INT32,chaos::DataType::Input);
    addAttributeToDataSet("HEIGHT","Y image size",chaos::DataType::TYPE_INT32,chaos::DataType::Input);
    addAttributeToDataSet("OFFSETX","AreaOfInterest Offset X (if supported)",chaos::DataType::TYPE_INT32,chaos::DataType::Input);
    addAttributeToDataSet("OFFSETY","AreaOfInterest Offset Y (if supported)",chaos::DataType::TYPE_INT32,chaos::DataType::Input);
    addAttributeToDataSet("DEPTH","Pixel depth",chaos::DataType::TYPE_INT32,chaos::DataType::Input);
    addAttributeToDataSet("SHUTTER","Shutter % (0:100, -1 = Auto) Adjust the exposure of the camera",chaos::DataType::TYPE_INT32,chaos::DataType::Input);
    addAttributeToDataSet("BRIGHTNESS","Brightness %(0:100, -1 = Auto) Adjusting the brightness lightens or darkens the entire image.",chaos::DataType::TYPE_INT32,chaos::DataType::Input);
    addAttributeToDataSet("CONTRAST","Contrast % (0:100, -1 = Auto) Adjusting the contrast increases the difference between light and dark areas in the image.",chaos::DataType::TYPE_INT32,chaos::DataType::Input);

    addAttributeToDataSet("SHARPNESS","Sharpness % (0:100, -1 = Auto) Amount of sharpening to apply. The higher the sharpness, the more distinct the image subject's contours will be",chaos::DataType::TYPE_INT32,chaos::DataType::Input);
    addAttributeToDataSet("GAIN","Gain % (0:100, -1 = Auto)",chaos::DataType::TYPE_INT32,chaos::DataType::Input);
    addAttributeToDataSet("MODE","0=grabbing,1=triggered,2=pulse",chaos::DataType::TYPE_BOOLEAN,chaos::DataType::Input);
    addAttributeToDataSet("FMT","image format (jpg,png,gif...)",chaos::DataType::TYPE_STRING,chaos::DataType::Input);
    addAttributeToDataSet("FILTER","Filter type and parameters (if any) ",chaos::DataType::TYPE_CLUSTER,chaos::DataType::Input);

    addBinaryAttributeAsSubtypeToDataSet("FRAMEBUFFER","image",chaos::DataType::SUB_TYPE_CHAR,DEFAULT_RESOLUTION,chaos::DataType::Output);

    addStateVariable(StateVariableTypeAlarmDEV,"operation_not_supported",
            "The operation is not supported by the current HW");
    addStateVariable(StateVariableTypeAlarmDEV,"camera_error",
            "Generic Camera Error");
    addStateVariable(StateVariableTypeAlarmCU,"encode_error",
            "Encoding error");
    addHandlerOnInputAttributeName< ::driver::sensor::camera::RTCameraBase, int >(this,
            &::driver::sensor::camera::RTCameraBase::setProp,
            "GAIN");
    addHandlerOnInputAttributeName< ::driver::sensor::camera::RTCameraBase, int >(this,
            &::driver::sensor::camera::RTCameraBase::setProp,
            "SHARPNESS");
    addHandlerOnInputAttributeName< ::driver::sensor::camera::RTCameraBase, int >(this,
            &::driver::sensor::camera::RTCameraBase::setProp,
            "BRIGHTNESS");
    addHandlerOnInputAttributeName< ::driver::sensor::camera::RTCameraBase, int >(this,
            &::driver::sensor::camera::RTCameraBase::setProp,
            "SHUTTER");
    addHandlerOnInputAttributeName< ::driver::sensor::camera::RTCameraBase, int >(this,
            &::driver::sensor::camera::RTCameraBase::setProp,
            "WIDTH");
    addHandlerOnInputAttributeName< ::driver::sensor::camera::RTCameraBase, int >(this,
            &::driver::sensor::camera::RTCameraBase::setProp,
            "HEIGHT");

    addHandlerOnInputAttributeName< ::driver::sensor::camera::RTCameraBase, int >(this,
            &::driver::sensor::camera::RTCameraBase::setProp,
            "OFFSETX");
    addHandlerOnInputAttributeName< ::driver::sensor::camera::RTCameraBase, int >(this,
            &::driver::sensor::camera::RTCameraBase::setProp,
            "OFFSETY");
}


//!Define custom control unit attribute
void RTCameraBase::unitDefineCustomAttribute() {

}

//!Initialize the Custom Control Unit
void RTCameraBase::unitInit() throw(chaos::CException) {
    int ret;
    AttributeSharedCacheWrapper * cc=getAttributeCache();
    driver=new CameraDriverInterface(getAccessoInstanceByIndex(0));

    //breanch number and soft reset

    sizex=cc->getROPtr<int32_t>(DOMAIN_INPUT, "WIDTH");
    sizey=cc->getROPtr<int32_t>(DOMAIN_INPUT, "HEIGHT");
    offsetx=cc->getROPtr<int32_t>(DOMAIN_INPUT, "OFFSETX");
    offsety=cc->getROPtr<int32_t>(DOMAIN_INPUT, "OFFSETY");

    if(*sizex == 0 || *sizey == 0){
        RTCameraBaseLERR_<<"Invalid image sizes:"<<*sizex<<"x"<<*sizey;
    }
    setProp("WIDTH",*sizex,0);
    setProp("HEIGHT",*sizey,0);
    setProp("OFFSETX",*offsetx,0);
    setProp("OFFSETY",*offsety,0);

    depth=cc->getROPtr<int32_t>(DOMAIN_INPUT, "DEPTH");
    mode=cc->getROPtr<int32_t>(DOMAIN_INPUT, "MODE");
    framebuf_out=cc->getRWPtr<uint8_t>(DOMAIN_OUTPUT, "FRAMEBUFFER");
    fmt=cc->getRWPtr<char>(DOMAIN_INPUT, "FMT");
    cc->setOutputAttributeNewSize("FRAMEBUFFER",*sizex*(*sizey)*(std::max(*depth/8,1)),true);
    if((ret=driver->cameraInit(0,0))!=0){
        throw chaos::CException(ret,"cannot initialize camera",__PRETTY_FUNCTION__);
    }
    int size=*sizex*(*sizey)*(std::max(*depth/8,1));

    framebuf = (uint8_t*)malloc(size);
    RTCameraBaseLDBG_<<"Starting acquiring imagex "<<*sizex<<"x"<<*sizey<<" depth:"<<*depth<<" bits";
    if(*fmt == 0){
        strcpy(fmt,".png");
    }
}

//!Execute the work, this is called with a determinated delay, it must be as fast as possible
void RTCameraBase::unitStart() throw(chaos::CException) {

  driver->startGrab(0,framebuf,NULL);
}

//!Execute the Control Unit work
void RTCameraBase::unitRun() throw(chaos::CException) {
  //get the output attribute pointer form the internal cache
    int ret;
    int size=*sizex*(*sizey)*(std::max(*depth/8,1));
    ret=driver->waitGrab(5000);
    RTCameraBaseLDBG_<<" raw image read:"<<ret;
     cv::Mat image(*sizey,*sizex,CV_8UC1,framebuf);
    if(image.empty()){
          RTCameraBaseLERR_"cannot convert image";
           return;
     }
    std::vector<uchar> buf;
    bool code = cv::imencode(fmt, image, buf );
    if(code==false){
        setStateVariableSeverity(StateVariableTypeAlarmCU,"encode_error", chaos::common::alarm::MultiSeverityAlarmLevelHigh);

    } else {
        setStateVariableSeverity(StateVariableTypeAlarmCU,"encode_error", chaos::common::alarm::MultiSeverityAlarmLevelClear);

    }
    uchar* result = reinterpret_cast<uchar*> (&buf[0]);
    RTCameraBaseLDBG_<<" jpeg image encoded:"<<buf.size();
    getAttributeCache()->setOutputAttributeNewSize("FRAMEBUFFER", buf.size());

    memcpy(framebuf_out,result,buf.size());
    //    namedWindow("Captura",WINDOW_AUTOSIZE);
    //    imshow( "Captura", image );


      getAttributeCache()->setOutputDomainAsChanged();
}

//!Execute the Control Unit work
void RTCameraBase::unitStop() throw(chaos::CException) {

}

//!Deinit the Control Unit
void RTCameraBase::unitDeinit() throw(chaos::CException) {
    if(framebuf){
        free(framebuf);
        framebuf=0;
    }
}

//! pre imput attribute change
void RTCameraBase::unitInputAttributePreChangeHandler() throw(chaos::CException) {

}

//! attribute changed handler
void RTCameraBase::unitInputAttributeChangedHandler() throw(chaos::CException) {
//    std::vector<VariableIndexType> changed;
//    std::vector<VariableIndexType>::iterator j;
//    getAttributeCache()->getChangedInputAttributeIndex(changed);
//    //RTCameraBaseLAPP_<<"UnitRun";
    
//    for(j=changed.begin();j!=changed.end();j++){
//        const char** buffer;
        
//        getAttributeCache()->getReadonlyCachedAttributeValue<char>(DOMAIN_INPUT, *j, &buffer);
//        if(driver->writeChannel(buffer,*j,input_size[*j])){
//            RTCameraBaseLDBG_<<"writing output channel "<<*j<<", size :"<<input_size[*j];
//        }
//    }
//    getAttributeCache()->resetChangedInputIndex();


}

/*
CDataWrapper *RTCameraBase::my_custom_action(CDataWrapper *actionParam, bool& detachParam) {
	CDataWrapper *result =  new CDataWrapper();
	return result;
}
*/
