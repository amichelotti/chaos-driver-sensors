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

/*#define RTCameraBaseLAPP_		LAPP_ << "[RTCameraBase] "
#define RTCameraBaseLDBG_		LDBG_ << "[RTCameraBase] " << __PRETTY_FUNCTION__ << " "
#define RTCameraBaseLERR_		LERR_ << "[RTCameraBase] " << __PRETTY_FUNCTION__ << "("<<__LINE__<<") "
*/
#define RTCameraBaseLAPP_		CUAPP
#define RTCameraBaseLDBG_		CUDBG
#define RTCameraBaseLERR_		CUERR
/*
 Construct
 */
#define DECODE_CVENCODING(e,cvenc) \
    if(e == #cvenc){\
    RTCameraBaseLDBG_<<"Found Encoding:"<<#cvenc;\
    framebuf_encoding=cvenc;\
}

RTCameraBase::RTCameraBase(const string& _control_unit_id, const string& _control_unit_param, const ControlUnitDriverList& _control_unit_drivers):
RTAbstractControlUnit(_control_unit_id, _control_unit_param, _control_unit_drivers),framebuf_encoding(CV_8UC3) {
  RTCameraBaseLDBG_<<"Creating "<<_control_unit_id<<" params:"<<_control_unit_param;
  try{
      framebuf=NULL;
      chaos::common::data::CDataWrapper p;
      p.setSerializedJsonData(_control_unit_param.c_str());
      if(p.hasKey("FRAMEBUFFER_ENCODING")&&p.isStringValue("FRAMEBUFFER_ENCODING")){
          std::string enc=p.getStringValue("FRAMEBUFFER_ENCODING");
          DECODE_CVENCODING(enc,CV_8UC4);
          DECODE_CVENCODING(enc,CV_8UC3);
          DECODE_CVENCODING(enc,CV_8UC2);
          DECODE_CVENCODING(enc,CV_8UC1);
          DECODE_CVENCODING(enc,CV_8SC1);
          DECODE_CVENCODING(enc,CV_8SC2);
          DECODE_CVENCODING(enc,CV_8SC3);
          DECODE_CVENCODING(enc,CV_8SC4);

          DECODE_CVENCODING(enc,CV_16UC4);
          DECODE_CVENCODING(enc,CV_16UC3);
          DECODE_CVENCODING(enc,CV_16UC2);
          DECODE_CVENCODING(enc,CV_16UC1);
          DECODE_CVENCODING(enc,CV_16SC1);
          DECODE_CVENCODING(enc,CV_16SC2);
          DECODE_CVENCODING(enc,CV_16SC3);
          DECODE_CVENCODING(enc,CV_16SC4);
          DECODE_CVENCODING(enc,CV_32SC1);
          DECODE_CVENCODING(enc,CV_32SC2);
          DECODE_CVENCODING(enc,CV_32SC3);
          DECODE_CVENCODING(enc,CV_32SC4);

      }
  } catch(...){

  }
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
void RTCameraBase::updateProperty(){
    std::vector<std::string> props;
    camera_props.getAllKey(props);
    AttributeSharedCacheWrapper * cc=getAttributeCache();

    for(std::vector<std::string>::iterator i = props.begin();i!=props.end();i++){

        if(camera_props.getValueType(*i)==chaos::DataType::TYPE_DOUBLE){
            double tmp,*p;
            driver->getCameraProperty(*i,tmp);
            p=cc->getRWPtr<double>(DOMAIN_OUTPUT,*i);
            *p=tmp;
        }
        if(camera_props.getValueType(*i)==chaos::DataType::TYPE_INT32){
            int32_t tmp,*p;
            driver->getCameraProperty(*i,tmp);
            p=cc->getRWPtr<int32_t>(DOMAIN_OUTPUT,*i);
            *p=tmp;
        }
    }
}
bool  RTCameraBase::setProp(const std::string &name, int32_t value, uint32_t size){
    int ret;
    RTCameraBaseLDBG_<<"SET IPROP:"<<name<<" VALUE:"<<value;
    ret=driver->setCameraProperty(name,value);
    setStateVariableSeverity(StateVariableTypeAlarmDEV,"operation_not_supported", chaos::common::alarm::MultiSeverityAlarmLevelClear);

    if(ret!=0){
        setStateVariableSeverity(StateVariableTypeAlarmDEV,"operation_not_supported", chaos::common::alarm::MultiSeverityAlarmLevelWarning);

    }
    updateProperty();

    return (ret==0);
}

bool  RTCameraBase::setProp(const std::string &name, double value, uint32_t size){
    int ret;
    RTCameraBaseLDBG_<<"SET FPROP:"<<name<<" VALUE:"<<value;

    ret=driver->setCameraProperty(name,value);
    setStateVariableSeverity(StateVariableTypeAlarmDEV,"operation_not_supported", chaos::common::alarm::MultiSeverityAlarmLevelClear);

    if(ret!=0){
        setStateVariableSeverity(StateVariableTypeAlarmDEV,"operation_not_supported", chaos::common::alarm::MultiSeverityAlarmLevelWarning);

    }
    updateProperty();
    return (ret==0);
}
//!Return the definition of the control unit
/*!
The api that can be called withi this method are listed into
"Control Unit Definition Public API" module into html documentation
(chaosframework/Documentation/html/group___control___unit___definition___api.html)
*/
void RTCameraBase::unitDefineActionAndDataset() throw(chaos::CException) {
    driver=new CameraDriverInterface(getAccessoInstanceByIndex(0));
    if(driver==NULL){
        throw chaos::CException(-1,"cannot create driver",__PRETTY_FUNCTION__);
    }
    if(driver->getCameraProperties(camera_props)!=0){
        throw chaos::CException(-1,"Error retrieving camera properties",__PRETTY_FUNCTION__);

    }
    std::vector<std::string> props;
    camera_props.getAllKey(props);
    addAttributeToDataSet("FMT","image format (jpg,png,gif...)",chaos::DataType::TYPE_STRING,chaos::DataType::Bidirectional);


    for(std::vector<std::string>::iterator i = props.begin();i!=props.end();i++){

        addAttributeToDataSet(*i,*i,camera_props.getValueType(*i),chaos::DataType::Input);
        addAttributeToDataSet(*i,*i,camera_props.getValueType(*i),chaos::DataType::Output);

        if(camera_props.getValueType(*i)==chaos::DataType::TYPE_DOUBLE){
            addHandlerOnInputAttributeName< ::driver::sensor::camera::RTCameraBase, double >(this,
                    &::driver::sensor::camera::RTCameraBase::setProp,
                    *i);
        }
        if(camera_props.getValueType(*i)==chaos::DataType::TYPE_INT32){
            addHandlerOnInputAttributeName< ::driver::sensor::camera::RTCameraBase, int32_t >(this,
                    &::driver::sensor::camera::RTCameraBase::setProp,
                    *i);
        }
    }
    addBinaryAttributeAsSubtypeToDataSet("FRAMEBUFFER","image",chaos::DataType::SUB_TYPE_CHAR,DEFAULT_RESOLUTION,chaos::DataType::Output);

}


//!Define custom control unit attribute
void RTCameraBase::unitDefineCustomAttribute() {

}

//!Initialize the Custom Control Unit
void RTCameraBase::unitInit() throw(chaos::CException) {
    int ret;
    int32_t itype;
    int32_t width,height;
    AttributeSharedCacheWrapper * cc=getAttributeCache();
    // this properties must exist
    //
    sizex=cc->getRWPtr<int32_t>(DOMAIN_INPUT, "WIDTH");
    sizey=cc->getRWPtr<int32_t>(DOMAIN_INPUT, "HEIGHT");
    //
    mode=cc->getROPtr<int32_t>(DOMAIN_INPUT, "TRIGGER_MODE");
    framebuf_out=cc->getRWPtr<uint8_t>(DOMAIN_OUTPUT, "FRAMEBUFFER");
    fmt=cc->getRWPtr<char>(DOMAIN_INPUT, "FMT");
    ofmt=cc->getRWPtr<char>(DOMAIN_OUTPUT, "FMT");
    std::vector<std::string> props;
    camera_props.getAllKey(props);
    //breanch number and soft reset
    for(std::vector<std::string>::iterator i = props.begin();i!=props.end();i++){

        if(camera_props.getValueType(*i)==chaos::DataType::TYPE_DOUBLE){
            double tmp=cc->getValue<double>(DOMAIN_INPUT,*i);
            setProp(*i,tmp,0);

        }
        if(camera_props.getValueType(*i)==chaos::DataType::TYPE_INT32){
            int32_t tmp=cc->getValue<int32_t>(DOMAIN_INPUT,*i);
            setProp(*i,tmp,0);
        }
    }


    if((ret=driver->cameraInit(0,0))!=0){
        throw chaos::CException(ret,"cannot initialize camera",__PRETTY_FUNCTION__);
    }



    for(std::vector<std::string>::iterator i = props.begin();i!=props.end();i++){

        if(camera_props.getValueType(*i)==chaos::DataType::TYPE_DOUBLE){
            double tmp,*p;
            driver->getCameraProperty(*i,tmp);
            p=cc->getRWPtr<double>(DOMAIN_INPUT,*i);
            *p=tmp;
        }
        if(camera_props.getValueType(*i)==chaos::DataType::TYPE_INT32){
            int32_t tmp,*p;
            driver->getCameraProperty(*i,tmp);
            p=cc->getRWPtr<int32_t>(DOMAIN_INPUT,*i);
            *p=tmp;
        }
    }

    if(driver->getImageProperties(width,height,itype)==0){
        RTCameraBaseLDBG_<<"CAMERA IMAGE:"<<width<<"x"<<height;
        *sizex=width;
        *sizey=height;
    }
    if((*sizex == 0) || (*sizey == 0)){
        RTCameraBaseLERR_<<"Invalid image sizes:"<<*sizex<<"x"<<*sizey;
        throw chaos::CException(-3,"Invalid image size!!",__PRETTY_FUNCTION__);
    }

    cc->setOutputAttributeNewSize("FRAMEBUFFER",*sizex*(*sizey)*(4),true);

    int size=*sizex*(*sizey)*4;

    framebuf = (uint8_t*)malloc(size);
    RTCameraBaseLDBG_<<"Starting acquiring imagex "<<*sizex<<"x"<<*sizey;
    if(*fmt == 0){
        strcpy(encoding,".png");

    } else {
        snprintf(encoding,sizeof(encoding),".%s",fmt);
    }
    snprintf(ofmt,sizeof(encoding),encoding);
    getAttributeCache()->setInputDomainAsChanged();
    getAttributeCache()->setOutputDomainAsChanged();


}

//!Execute the work, this is called with a determinated delay, it must be as fast as possible
void RTCameraBase::unitStart() throw(chaos::CException) {
  updateProperty();

  driver->startGrab(0,framebuf,NULL);
}

//!Execute the Control Unit work
void RTCameraBase::unitRun() throw(chaos::CException) {
  //get the output attribute pointer form the internal cache
    int ret;
  //  int size=*sizex*(*sizey)*(std::max(*depth/8,1));
    ret=driver->waitGrab(5000);
    if(ret<0){
        RTCameraBaseLERR_<<" Timeout!!";
        return;
    }
    cv::Mat image(*sizey,*sizex,framebuf_encoding,framebuf);
    if(image.empty()){
          RTCameraBaseLERR_"cannot convert image";
           return;
     }
    std::vector<uchar> buf;
    try {
        bool code = cv::imencode(encoding, image, buf );
        if(code==false){
            setStateVariableSeverity(StateVariableTypeAlarmCU,"encode_error", chaos::common::alarm::MultiSeverityAlarmLevelHigh);
            LERR_<<"Encode error:"<<fmt;
            return;

        } else {
            setStateVariableSeverity(StateVariableTypeAlarmCU,"encode_error", chaos::common::alarm::MultiSeverityAlarmLevelClear);

        }
    } catch(...){
         setStateVariableSeverity(StateVariableTypeAlarmCU,"encode_error", chaos::common::alarm::MultiSeverityAlarmLevelHigh);
         LERR_<<"Encode exception:"<<fmt;
         return;
    }


    uchar* result = reinterpret_cast<uchar*> (&buf[0]);
    //getAttributeCache()->setOutputAttributeNewSize("FRAMEBUFFER", buf.size());
    getAttributeCache()->setOutputAttributeValue("FRAMEBUFFER",result,buf.size());
    //memcpy(framebuf_out,result,buf.size());
    //    namedWindow("Captura",WINDOW_AUTOSIZE);
    //    imshow( "Captura", image );


    getAttributeCache()->setOutputDomainAsChanged();
    RTCameraBaseLDBG_<<encoding <<" image encoded:"<<buf.size();

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
    driver->cameraDeinit();
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
