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
RTAbstractControlUnit(_control_unit_id, _control_unit_param, _control_unit_drivers),driver(NULL),framebuf_encoding(CV_8UC3),captureImg(CAMERA_FRAME_BUFFERING),encodedImg(CAMERA_FRAME_BUFFERING),encode_time(0),capture_time(0),network_time(0),captureWritePointer(0),encodeWritePointer(0), hw_trigger_timeout_us(5000000),sw_trigger_timeout_us(0),trigger_timeout(0) {
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
      if(p.hasKey(TRIGGER_HW_TIMEOUT_KEY)){
            hw_trigger_timeout_us=p.getInt32Value(TRIGGER_HW_TIMEOUT_KEY);
      }
      if(p.hasKey(TRIGGER_SW_TIMEOUT_KEY)){
            sw_trigger_timeout_us=p.getInt32Value(TRIGGER_SW_TIMEOUT_KEY);
      }
       for(int cnt=0;cnt<CAMERA_FRAME_BUFFERING;cnt++){
        framebuf_out[cnt].buf=NULL;
        framebuf_out[cnt].size=0;
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
    if(driver->getCameraProperties(camera_props)!=0){
        throw chaos::CException(-1,"Error retrieving camera properties",__PRETTY_FUNCTION__);

    }
    camera_props.getAllKey(props);
    AttributeSharedCacheWrapper * cc=getAttributeCache();

    for(std::vector<std::string>::iterator i = props.begin();i!=props.end();i++){
        if(!camera_props.isCDataWrapperValue(*i)){

        if(camera_props.getValueType(*i)==chaos::DataType::TYPE_DOUBLE){
            double tmp,*p;
            driver->getCameraProperty(*i,tmp);
            RTCameraBaseLDBG_<<"Camera Property double "<<*i<<" VALUE:"<<tmp;

            p=cc->getRWPtr<double>(DOMAIN_OUTPUT,*i);
            *p=tmp;
        }
        if(camera_props.getValueType(*i)==chaos::DataType::TYPE_INT32){
            int32_t tmp,*p;
            driver->getCameraProperty(*i,tmp);
            RTCameraBaseLDBG_<<"Camera Property int "<<*i<<" VALUE:"<<tmp;

            p=cc->getRWPtr<int32_t>(DOMAIN_OUTPUT,*i);
            *p=tmp;
        }
        }
    }
}
bool  RTCameraBase::setProp(const std::string &name, int32_t value, uint32_t size){
    int ret;
    int32_t valuer;
    RTCameraBaseLDBG_<<"SET IPROP:"<<name<<" VALUE:"<<value;
    bool stopgrab=(stopCapture==false)&&
                    (((name == WIDTH_KEY)&&(value !=*sizex)) || 
                    ((name == HEIGHT_KEY)&&(value !=*sizey)) ||
                    ((name == OFFSETX_KEY)&&(value !=*offsetx)) ||
                    ((name == OFFSETY_KEY)&&(value !=*offsety)));
    if(stopgrab){
        stopGrabbing();
    }
    ret=driver->setCameraProperty(name,value);
    setStateVariableSeverity(StateVariableTypeAlarmDEV,"operation_not_supported", chaos::common::alarm::MultiSeverityAlarmLevelClear);

    if(ret!=0){
        std::stringstream ss;
        ss<<"error setting \""<<name<<"\" to "<<value;
        setStateVariableSeverity(StateVariableTypeAlarmDEV,"operation_not_supported", chaos::common::alarm::MultiSeverityAlarmLevelWarning);
		metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelError,ss.str());

    }

   // updateProperty();
    driver->getCameraProperty(name,valuer);
    AttributeSharedCacheWrapper * cc=getAttributeCache();
    int32_t * p=cc->getRWPtr<int32_t>(DOMAIN_OUTPUT,name);
    *p=valuer;
    RTCameraBaseLDBG_<<"SET IPROP:"<<name<<" SET VALUE:"<<value<<" READ VALUE:"<<valuer<<" ret:"<<ret;
    if(stopgrab){
        startGrabbing();
    }
    getAttributeCache()->setInputDomainAsChanged();
    getAttributeCache()->setOutputDomainAsChanged();
    return (ret==0);
}

bool  RTCameraBase::setProp(const std::string &name, double value, uint32_t size){
    int ret;
    double valuer;
    RTCameraBaseLDBG_<<"SET FPROP:"<<name<<" VALUE:"<<value<<" READ:"<<valuer<<" ret:"<<ret;

    ret=driver->setCameraProperty(name,value);
    setStateVariableSeverity(StateVariableTypeAlarmDEV,"operation_not_supported", chaos::common::alarm::MultiSeverityAlarmLevelClear);

    if(ret!=0){
        setStateVariableSeverity(StateVariableTypeAlarmDEV,"operation_not_supported", chaos::common::alarm::MultiSeverityAlarmLevelWarning);

    }
    //updateProperty();
     driver->getCameraProperty(name,valuer);
    AttributeSharedCacheWrapper * cc=getAttributeCache();
    double * p=cc->getRWPtr<double>(DOMAIN_OUTPUT,name);
    *p=valuer;

     getAttributeCache()->setInputDomainAsChanged();
    getAttributeCache()->setOutputDomainAsChanged();

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
    addAttributeToDataSet("CAPTURE_FRAMERATE","Capture Frame Rate",chaos::DataType::TYPE_INT32,chaos::DataType::Output);
    addAttributeToDataSet("ENCODE_FRAMERATE","Encode Frame Rate",chaos::DataType::TYPE_INT32,chaos::DataType::Output);

    for(std::vector<std::string>::iterator i = props.begin();i!=props.end();i++){
        if(!camera_props.isCDataWrapperValue(*i)){
        RTCameraBaseLDBG_<<"ADDING ATTRIBUTE:"<<*i<<" Type:"<<camera_props.getValueType(*i);

        addAttributeToDataSet(*i,*i,camera_props.getValueType(*i),chaos::DataType::Bidirectional);
       // getAttributeCache()->addCustomAttribute(*i, 8, camera_props.getValueType(*i));
       // getAttributeCache()->setCustomAttributeValue("quit", &quit, sizeof(bool));
       // addCustomAttribute(*i,sizeof(double),camera_props.getValueType(*i));
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
    }
    addBinaryAttributeAsSubtypeToDataSet("FRAMEBUFFER","image",chaos::DataType::SUB_TYPE_CHAR,DEFAULT_RESOLUTION,chaos::DataType::Output);
    addStateVariable(StateVariableTypeAlarmDEV,"operation_not_supported","the operation in not supported i.e property not present or accesible");
    addStateVariable(StateVariableTypeAlarmCU,"encode_error","an error occurred during encode");
    addStateVariable(StateVariableTypeAlarmDEV,"capture_error","an error occurred during capture");
    addStateVariable(StateVariableTypeAlarmDEV,"capture_timeout","a timeout has occurred");


}


//!Define custom control unit attribute
void RTCameraBase::unitDefineCustomAttribute() {
    std::string config;
    chaos::common::data::CDataWrapper attr;
    getAttributeCache()->addCustomAttribute("config", 8192, chaos::DataType::TYPE_CLUSTER);
    if(driver->getCameraProperties(attr)!=0){
        throw chaos::CException(-1,"Error retrieving camera properties",__PRETTY_FUNCTION__);

    }
    config=attr.getCompliantJSONString();
    RTCameraBaseLDBG_<<"ADDING CONFIG:"<<config;
    getAttributeCache()->setCustomAttributeValue("config", (void*)config.c_str(),config.size());
    getAttributeCache()->setCustomDomainAsChanged();
    pushCustomDataset();    
   
}



//!Initialize the Custom Control Unit
void RTCameraBase::unitInit() throw(chaos::CException) {
    int ret;
    int32_t itype;
    int32_t width,height;
    AttributeSharedCacheWrapper * cc=getAttributeCache();
    // this properties must exist
    //
    sizex=cc->getRWPtr<int32_t>(DOMAIN_INPUT, WIDTH_KEY);
    sizey=cc->getRWPtr<int32_t>(DOMAIN_INPUT, HEIGHT_KEY);
    offsetx=cc->getRWPtr<int32_t>(DOMAIN_INPUT, OFFSETX_KEY);
    offsety=cc->getRWPtr<int32_t>(DOMAIN_INPUT, OFFSETY_KEY);
    //
    mode=cc->getROPtr<int32_t>(DOMAIN_INPUT, TRIGGER_MODE_KEY);
 //   camera_out=cc->getRWPtr<uint8_t>(DOMAIN_OUTPUT, "FRAMEBUFFER");
    fmt=cc->getRWPtr<char>(DOMAIN_INPUT, "FMT");
    ofmt=cc->getRWPtr<char>(DOMAIN_OUTPUT, "FMT");
    capture_frame_rate=cc->getRWPtr<int32_t>(DOMAIN_OUTPUT, "CAPTURE_FRAMERATE");
    enc_frame_rate=cc->getRWPtr<int32_t>(DOMAIN_OUTPUT, "ENCODE_FRAMERATE");
    std::vector<std::string> props;
    camera_props.getAllKey(props);
    //breanch number and soft reset
    


    if((ret=driver->cameraInit(0,0))!=0){
        throw chaos::CException(ret,"cannot initialize camera",__PRETTY_FUNCTION__);
    }

    for(std::vector<std::string>::iterator i = props.begin();i!=props.end();i++){
        if(!camera_props.isCDataWrapperValue(*i)){

        if(camera_props.getValueType(*i)==chaos::DataType::TYPE_DOUBLE){
            double tmp=cc->getValue<double>(DOMAIN_INPUT,*i);
            RTCameraBaseLDBG_<<"Init Double \""<<*i<<"\" from input:"<<tmp;

            setProp(*i,tmp,0);

        }
        if(camera_props.getValueType(*i)==chaos::DataType::TYPE_INT32){
            int32_t tmp=cc->getValue<int32_t>(DOMAIN_INPUT,*i);
            RTCameraBaseLDBG_<<"Init Integer \""<<*i<<"\" from input:"<<tmp;

            setProp(*i,tmp,0);
        }
        }
    }

    for(std::vector<std::string>::iterator i = props.begin();i!=props.end();i++){
        if(!camera_props.isCDataWrapperValue(*i)){

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

   // framebuf = (uint8_t*)malloc(size);
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
void RTCameraBase::cameraGrabCallBack(const void*buf,uint32_t blen,uint32_t width,uint32_t heigth, uint32_t error){
   
}
void RTCameraBase::startGrabbing(){
     RTCameraBaseLDBG_<<"Start Grabbing";
    metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelInfo,"Start grabbing");

    // allocate buffers;
    encodeWritePointer=0;
    captureWritePointer=0;
    for(int cnt=0;cnt<CAMERA_FRAME_BUFFERING;cnt++){
        framebuf_out[cnt].buf=(unsigned char*)malloc(*sizex * *sizey * 4);
        framebuf_out[cnt].size=*sizex * *sizey * 4;
    }
    RTCameraBaseLDBG_<<"Allocated "<<CAMERA_FRAME_BUFFERING<<" buffers of "<<(*sizex * *sizey * 4)<<" bytes";
    updateProperty();
    driver->startGrab(0);
    stopCapture=false;

    capture_th=boost::thread(&RTCameraBase::captureThread,this);
    getAttributeCache()->setCustomDomainAsChanged()
    pushCustomDataset();
}

void RTCameraBase::stopGrabbing(){
    stopCapture=true;
    metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelInfo,"Stop grabbing");

    RTCameraBaseLDBG_<<"Stop Grabbing..."<<stopCapture;

    driver->stopGrab();
    wait_capture.notify_all();
    wait_encode.notify_all();
    full_encode.notify_all();
    full_capture.notify_all();
    capture_th.join();
    encode_th.join();
     for(int cnt=0;cnt<CAMERA_FRAME_BUFFERING;cnt++){
        free(framebuf_out[cnt].buf);
        framebuf_out[cnt].size=0;
        framebuf_out[cnt].buf=NULL;
    }
    encodedImg.consume_all([](buf_t i){ });
    captureImg.consume_all([](buf_t i){ });
    setStateVariableSeverity(StateVariableTypeAlarmDEV,"operation_not_supported", chaos::common::alarm::MultiSeverityAlarmLevelClear);
    setStateVariableSeverity(StateVariableTypeAlarmDEV,"capture_error", chaos::common::alarm::MultiSeverityAlarmLevelClear);
    setStateVariableSeverity(StateVariableTypeAlarmDEV,"capture_timeout", chaos::common::alarm::MultiSeverityAlarmLevelClear);
    setStateVariableSeverity(StateVariableTypeAlarmCU,"encode_error", chaos::common::alarm::MultiSeverityAlarmLevelClear);

    RTCameraBaseLDBG_<<"Stop Grabbing done"<<stopCapture;

}
//!Execute the work, this is called with a determinated delay, it must be as fast as possible
void RTCameraBase::unitStart() throw(chaos::CException) {
   startGrabbing();
}

void RTCameraBase::captureThread(){
    int ret;
    const char*img=0;
    uint64_t start;
    counter_capture=0;
    RTCameraBaseLDBG_<<"Capture thread STARTED";
    encode_th=boost::thread(&RTCameraBase::encodeThread,this);

    while(!stopCapture){
        img=0;
         start=chaos::common::utility::TimingUtil::getTimeStampInMicroseconds();
          ret=driver->waitGrab(&img,trigger_timeout);
          if((img!=0) && (ret>0)){
            setStateVariableSeverity(StateVariableTypeAlarmDEV,"capture_error", chaos::common::alarm::MultiSeverityAlarmLevelClear);
            setStateVariableSeverity(StateVariableTypeAlarmDEV,"capture_timeout", chaos::common::alarm::MultiSeverityAlarmLevelClear);

            if(captureWritePointer>=CAMERA_FRAME_BUFFERING){
                captureWritePointer=0;
            }
            if(framebuf_out[captureWritePointer].size<ret){
                framebuf_out[captureWritePointer].buf=(unsigned char*)realloc(framebuf_out[captureWritePointer].buf,ret);
                if(framebuf_out[captureWritePointer].buf==NULL){
                    throw chaos::CException(-1,"Heap memory exahusted",__PRETTY_FUNCTION__);
                }
                framebuf_out[captureWritePointer].size=ret;
            }
            memcpy((void*)framebuf_out[captureWritePointer].buf,img,ret);
            capture_time+=(chaos::common::utility::TimingUtil::getTimeStampInMicroseconds()-start);
            counter_capture++;
           
            if(captureImg.push(framebuf_out[captureWritePointer])==false){
                // FULL
                RTCameraBaseLDBG_<<"Capture FULL write pointer:"<<captureWritePointer;
                wait_capture.notify_one();

                boost::mutex::scoped_lock lock(mutex_io);
                full_capture.wait(lock);
            } else {
                RTCameraBaseLDBG_<<"Capture write pointer:"<<captureWritePointer<<" read pointer:"<<encodeWritePointer;
                captureWritePointer++;
                wait_capture.notify_one();

            }
        } else {

            if(ret==TRIGGER_TIMEOUT_ERROR){
                RTCameraBaseLERR_<<" wait returned:"<<ret<<" timeout stopped:"<<stopCapture ;
                setStateVariableSeverity(StateVariableTypeAlarmDEV,"capture_timeout", chaos::common::alarm::MultiSeverityAlarmLevelWarning);

            } else {
                RTCameraBaseLERR_<<" wait returned:"<<ret<<" buffer  is bypass ? capture stopped:"<<stopCapture ;
                setStateVariableSeverity(StateVariableTypeAlarmDEV,"capture_error", chaos::common::alarm::MultiSeverityAlarmLevelHigh);

            }
            usleep(100000);

        }
    }
    captureWritePointer=0;

    RTCameraBaseLDBG_<<"Capture thread ENDED";

}
void RTCameraBase::encodeThread(){
    uint64_t start;
    counter_encode=0;
    RTCameraBaseLDBG_<<"Encode thread STARTED";
    while(!stopCapture){ 
        if(encodeWritePointer>=CAMERA_FRAME_BUFFERING){
            encodeWritePointer=0;
        }
        buf_t a;

        if(captureImg.pop(a)){
            start=chaos::common::utility::TimingUtil::getTimeStampInMicroseconds();
            full_capture.notify_one();

        cv::Mat image(*sizey,*sizex,framebuf_encoding,a.buf);
        try {
        //bool code = cv::imencode(encoding, image, buf );
           

            bool code = cv::imencode(encoding, image,  encbuf[encodeWritePointer]);
            if(code==false){
                setStateVariableSeverity(StateVariableTypeAlarmCU,"encode_error", chaos::common::alarm::MultiSeverityAlarmLevelHigh);
                LERR_<<"Encode error:"<<fmt;
                

            } else {
                setStateVariableSeverity(StateVariableTypeAlarmCU,"encode_error", chaos::common::alarm::MultiSeverityAlarmLevelClear);
                buf_t a;
                a.buf= reinterpret_cast<uchar*> (&encbuf[encodeWritePointer][0]);
                a.size=encbuf[encodeWritePointer].size();
                wait_encode.notify_one();
                encode_time+=(chaos::common::utility::TimingUtil::getTimeStampInMicroseconds()-start);
                counter_encode++;
                if(encodedImg.push(a)==false){
                    // FULL
                    boost::mutex::scoped_lock lock(mutex_encode);
                    RTCameraBaseLDBG_<<"Encode FULL write pointer:"<<encodeWritePointer;
                    full_encode.wait(lock);

                } else {
                    encodeWritePointer++;
                }
               
            }
        } catch(...){
             setStateVariableSeverity(StateVariableTypeAlarmCU,"encode_error", chaos::common::alarm::MultiSeverityAlarmLevelHigh);
            LERR_<<"Encode exception:"<<fmt;
        }
        } else {
            RTCameraBaseLDBG_<<"Encode EMPTY";

            boost::mutex::scoped_lock lock(mutex_encode);
            wait_capture.wait(lock);
        }
    }
    encodeWritePointer=0;
    RTCameraBaseLDBG_<<"Encode thread ENDED";

}

//!Execute the Control Unit work
void RTCameraBase::unitRun() throw(chaos::CException) {
  //get the output attribute pointer form the internal cache
    if((encode_time >0) && (capture_time >0)){
        *enc_frame_rate=(1000000*counter_encode/encode_time);
        *capture_frame_rate=(1000000*counter_capture/capture_time);
    }
    
    if((counter_capture%100000)==0){
        //RTCameraBaseLDBG_<<"CAPTURE TIME (us):"<<capture_time/counter_capture<<" HZ:"<<(1000000*counter_capture/capture_time)<<" ENCODE TIME (us):"<<encode_time/counter_encode<<" HZ:"<<(1000000*counter_encode/encode_time);
        capture_time=0;
        encode_time=0;
        counter_encode=0;
        counter_capture=0;
    }
    
    buf_t a;
    if(encodedImg.pop(a)){
        RTCameraBaseLDBG_<<"Encode Write Pointer:"<<encodeWritePointer<<" CaptureWrite:"<<captureWritePointer;
        
        getAttributeCache()->setOutputAttributeValue("FRAMEBUFFER",a.buf,a.size);
        getAttributeCache()->setOutputDomainAsChanged();
        full_encode.notify_one();

    } else {
       // RTCameraBaseLDBG_<<"Encode EMPTY :"<<encodeWritePointer<<" CaptureWrite:"<<captureWritePointer;
        full_encode.notify_one();

        boost::mutex::scoped_lock lock(mutex_encode);
        wait_encode.wait(lock);
    }

}

//!Execute the Control Unit work
void RTCameraBase::unitStop() throw(chaos::CException) {
    stopGrabbing();
    
}

//!Deinit the Control Unit
void RTCameraBase::unitDeinit() throw(chaos::CException) {
   
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
