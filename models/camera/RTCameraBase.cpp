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
#include <chaos/cu_toolkit/control_manager/AttributeSharedCacheWrapper.h>
#include "../../core/AbstractSensorDriver.h"
#include "CameraDriverInterface.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#define ZERO_COPY
#ifdef CERN_ROOT
#include "../../core/rootGaussianImage2dFit.h"
#endif
/*
   IMWRITE_PNG_STRATEGY_DEFAULT      = 0,
    IMWRITE_PNG_STRATEGY_FILTERED     = 1,
    IMWRITE_PNG_STRATEGY_HUFFMAN_ONLY = 2,
    IMWRITE_PNG_STRATEGY_RLE          = 3,
    IMWRITE_PNG_STRATEGY_FIXED        = 4,
*/

#define DECODE_CVENCODING(e, cvenc)                   \
  if (e == #cvenc) {                                  \
    RTCameraBaseLDBG_ << "Found Encoding:" << #cvenc; \
    framebuf_encoding = cvenc;                        \
    switch (cvenc) {                                  \
      case CV_8UC4:                                   \
      case CV_8SC4:                                   \
        bpp = 4;                                      \
        break;                                        \
      case CV_8UC3:                                   \
      case CV_8SC3:                                   \
        bpp = 3;                                      \
        break;                                        \
      case CV_8UC2:                                   \
      case CV_8SC2:                                   \
        bpp = 2;                                      \
        break;                                        \
      case CV_8UC1:                                   \
      case CV_8SC1:                                   \
        bpp = 1;                                      \
        break;                                        \
      case CV_16UC1:                                  \
      case CV_16SC1:                                  \
        bpp = 2;                                      \
        break;                                        \
      case CV_16UC2:                                  \
      case CV_16SC2:                                  \
        bpp = 4;                                      \
        break;                                        \
      case CV_16UC3:                                  \
      case CV_16SC3:                                  \
        bpp = 6;                                      \
        break;                                        \
      case CV_32SC1:                                  \
        bpp = 4;                                      \
        break;                                        \
      case CV_32SC2:                                  \
        bpp = 8;                                      \
        break;                                        \
      case CV_32SC3:                                  \
        bpp = 16;                                     \
        break;                                        \
      default:                                        \
        bpp = 16;                                     \
    }                                                 \
  }
using namespace chaos;
using namespace chaos::common::data::cache;
using namespace chaos::common::direct_io;

using namespace chaos::cu::driver_manager::driver;
using namespace ::driver::sensor::camera;
using namespace chaos::cu::control_manager;
using namespace cv;
using namespace boost::interprocess;

PUBLISHABLE_CONTROL_UNIT_IMPLEMENTATION(RTCameraBase)

/*#define RTCameraBaseLAPP_		LAPP_ << "[RTCameraBase] "
#define RTCameraBaseLDBG_		LDBG_ << "[RTCameraBase] " <<
__PRETTY_FUNCTION__ << " " #define RTCameraBaseLERR_		LERR_ <<
"[RTCameraBase] " << __PRETTY_FUNCTION__ << "("<<__LINE__<<") "
*/

#define INFO INFO_LOG(RTCameraBase)
#define DBG DBG_LOG(RTCameraBase)
#define ERR ERR_LOG(RTCameraBase)

#define RTCameraBaseLAPP_ INFO << "[" << getDeviceID() << "] "
#define RTCameraBaseLDBG_ DBG << "[" << getDeviceID() << "] "
#define RTCameraBaseLERR_ ERR << "[" << getDeviceID() << "] "
/*
 Construct
 */

RTCameraBase::RTCameraBase(const string                &_control_unit_id,
                           const string                &_control_unit_param,
                           const ControlUnitDriverList &_control_unit_drivers,
                           const int                    _buffering)
    : RTAbstractControlUnit(_control_unit_id, _control_unit_param, _control_unit_drivers), encode_thds(ENCODE_THREADS),driver(NULL), streamer(NULL),capture_exited(true),capture_cnt(0),framebuf_encoding(CV_8UC3), buffering(_buffering), png_strategy(IMWRITE_PNG_STRATEGY_DEFAULT ),encode_time(0), capture_time(0), network_time(0), hw_trigger_timeout_us(5000000), sw_trigger_timeout_us(0), imagesizex(0), imagesizey(0), apply_resize(false), trigger_timeout(5000), bpp(3), stopCapture(true), subImage(NULL), performCalib(false), applyCalib(false),cameraStreamEnable(true), calibrationImages(1), applyReference(false), refenceThick(2), refenceR(0), refenceG(255), refenceB(0) {
  RTCameraBaseLDBG_ << "Creating " << _control_unit_id
                    << " params:" << _control_unit_param;

  framebuf = NULL;
  strcpy(encoding,".png");

  try {
    std::stringstream ss;
    
      streamName = _control_unit_id;
      replace(streamName.begin(), streamName.end(), '/', '_');
 
    if(HttpStreamManager::getInstance()->getRoot().size()){
      streamer=HttpStreamManager::getInstance();
      RTCameraBaseLDBG_<<"Publishing on "<<HttpStreamManager::getInstance()->getRoot()+streamName;
    }
    chaos::common::data::CDataWrapper p;
    p.setSerializedJsonData(_control_unit_param.c_str());
    compression_factor=1;


    if (p.hasKey(FRAMEBUFFER_ENCODING_KEY) &&
        p.isStringValue(FRAMEBUFFER_ENCODING_KEY)) {
      framebuf_encoding_s = p.getStringValue(FRAMEBUFFER_ENCODING_KEY);
      framebuf_encoding   = fmt2cv(framebuf_encoding_s);
      std::string enc;
      bpp = cv2fmt(framebuf_encoding, enc);
    }
    if (p.hasKey(TRIGGER_HW_TIMEOUT_KEY)) {
      hw_trigger_timeout_us = p.getInt32Value(TRIGGER_HW_TIMEOUT_KEY);
    }
    if (p.hasKey(TRIGGER_SW_TIMEOUT_KEY)) {
      sw_trigger_timeout_us = p.getInt32Value(TRIGGER_SW_TIMEOUT_KEY);
    }
    if (p.hasKey("IMAGESIZEX")) {
      imagesizex = p.getInt32Value("IMAGESIZEX");
    }
    if (p.hasKey("IMAGESIZEY")) {
      imagesizey = p.getInt32Value("IMAGESIZEY");
    }
    
    if (imagesizex > 0 && imagesizey > 0) {
      apply_resize = true;
      RTCameraBaseLDBG_ << "Camera will resize image to fit into: "
                        << imagesizex << "x" << imagesizey;
    }
  } catch (...) {
  }

  createProperty("compression_factor", compression_factor, "compression_factor", [](AbstractControlUnit *thi, const std::string &name, const chaos::common::data::CDataWrapper &p) -> chaos::common::data::CDWUniquePtr {
        chaos::common::data::CDWUniquePtr ret(new chaos::common::data::CDataWrapper());
        ret->addInt32Value(PROPERTY_VALUE_KEY,((RTCameraBase*)thi)->compression_factor);
        return ret; }, [](AbstractControlUnit *thi, const std::string &name, const chaos::common::data::CDataWrapper &p) -> chaos::common::data::CDWUniquePtr {
    RTCameraBase *t = (RTCameraBase *)thi;

    t->changeEncodingParam(p.getInt32Value(PROPERTY_VALUE_KEY));
    
    
    LDBG_ << "using '"<<t->encoding<<"', parameter:" << p.getInt32Value(PROPERTY_VALUE_KEY);
    return p.clone();
  });
  createProperty("encode_threads", encode_thds, "encode_threads", [](AbstractControlUnit *thi, const std::string &name, const chaos::common::data::CDataWrapper &p) -> chaos::common::data::CDWUniquePtr {
        chaos::common::data::CDWUniquePtr ret(new chaos::common::data::CDataWrapper());
        ret->addInt32Value(PROPERTY_VALUE_KEY,((RTCameraBase*)thi)->encode_thds);
        return ret; }, [](AbstractControlUnit *thi, const std::string &name, const chaos::common::data::CDataWrapper &p) -> chaos::common::data::CDWUniquePtr {
        RTCameraBase *t = (RTCameraBase *)thi;
        if (t->hasStopped() == false) {
            LERR_ << "cannot change encoding if not stopped";
            return p.clone();
        }
        
        t->encode_thds=p.getInt32Value(PROPERTY_VALUE_KEY);
        if(t->encode_thds>MAX_ENCODE_THREADS){
            t->encode_thds=MAX_ENCODE_THREADS;
        }

        chaos::common::data::CDWUniquePtr ret(new chaos::common::data::CDataWrapper());
        ret->addInt32Value(PROPERTY_VALUE_KEY,((RTCameraBase*)thi)->encode_thds);
        LDBG_ << t->getDeviceID()<< " setting encode_threads to '"<<t->encode_thds;

        return ret; 
  });
createProperty("png_strategy", png_strategy, "png_strategy", [](AbstractControlUnit *thi, const std::string &name, const chaos::common::data::CDataWrapper &p) -> chaos::common::data::CDWUniquePtr {
        chaos::common::data::CDWUniquePtr ret(new chaos::common::data::CDataWrapper());
        ret->addInt32Value(PROPERTY_VALUE_KEY,((RTCameraBase*)thi)->png_strategy);
        return ret; }, 
        [](AbstractControlUnit *thi, const std::string &name, const chaos::common::data::CDataWrapper &p) -> chaos::common::data::CDWUniquePtr {
    RTCameraBase *t = (RTCameraBase *)thi;
    t->png_strategy=p.getInt32Value(PROPERTY_VALUE_KEY);
    t->changeEncodingParam(t->compression_factor);
    
    
    LDBG_ << "using PNG strategy '"<<t->encoding<<"', parameter:" << t->png_strategy;
    return p.clone();
  });

  char ver[256];
  sprintf(ver,"%d.%d.%d",CV_VERSION_MAJOR,CV_VERSION_MINOR,CV_VERSION_REVISION);
  createProperty("opencv",std::string(ver));
  CREATE_CU_BOOL_PROP("referenceON", "referenceOn", applyReference, RTCameraBase);
  CREATE_CU_INT_PROP("referenceThick", "referenceThick", refenceThick, 1, 20, 1, RTCameraBase);
 /* CREATE_CU_INT_PROP("referenceR", "referenceR", refenceR, 0, 255, 1, RTCameraBase);
  CREATE_CU_INT_PROP("referenceG", "referenceG", refenceG, 0, 255, 1, RTCameraBase);
  CREATE_CU_INT_PROP("referenceB", "referenceB", refenceB, 0, 255, 1, RTCameraBase);
  */
  CREATE_CU_BOOL_PROP("performCalib", "performCalib", performCalib, RTCameraBase);

  CREATE_CU_INT_PROP("calibrationImages", "calibrationImages", calibrationImages, 0, 1000, 1, RTCameraBase);

  createProperty(
      "apply_calib", applyCalib, "apply_calib", [](AbstractControlUnit *thi, const std::string &name, const chaos::common::data::CDataWrapper &p) -> chaos::common::data::CDWUniquePtr {
        chaos::common::data::CDWUniquePtr ret(new chaos::common::data::CDataWrapper());
        ret->addBoolValue(PROPERTY_VALUE_KEY,((RTCameraBase*)thi)->applyCalib);
        return ret; }, [](AbstractControlUnit *thi, const std::string &name, const chaos::common::data::CDataWrapper &p) -> chaos::common::data::CDWUniquePtr { 
          ((RTCameraBase*)thi)->applyCalib=p.getBoolValue(PROPERTY_VALUE_KEY);
          if(((RTCameraBase*)thi)->applyCalib){
              chaos::common::data::CDWUniquePtr calibrazione=  ((RTCameraBase*)thi)->loadData("calibration_image");
              if(calibrazione.get()){
                uint32_t size;
                const char * buf=calibrazione->getBinaryValue("FRAMEBUFFER",size);
                std::vector<uchar> data = std::vector<uchar>(buf, buf + size);
                cv::Mat sub=cv::imdecode(data,IMREAD_ANYCOLOR | IMREAD_ANYDEPTH);
                if (sub.data) {
                  cv::Mat *t=((RTCameraBase*)thi)->subImage;
                  ((RTCameraBase*)thi)->subImage = new cv::Mat(sub);

                  if(t){
                     delete t;
                  }
                  LDBG_ << "Apply Calib  Loading CALIB DATA "<<size;
                } else {
                  LERR_ << " CALIB DATA does not exists";
                }
              }

          }
          return p.clone(); });

          
}

/*
 Destructor
 */
RTCameraBase::~RTCameraBase() {
  if (driver) {
    driver->cameraDeinit();
    delete driver;
    driver = NULL;
  }
  if (subImage) {
    delete subImage;
  }
}

cv::Mat RTCameraBase::getMean(const std::vector<cv::Mat*> &images) {
  if (images.empty()) return cv::Mat();
  cv::Mat m=*images[0];
  //Mat m = Mat::zeros(images[0]->size(), CV_32F); //larger depth to avoid saturation
  // Create a 0 initialized image to use as accumulator
 // cv::Mat m(images[0]->rows, images[0]->cols, CV_64FC3);
  //m.setTo(Scalar(0, 0, 0, 0));

  // Use a temp image to hold the conversion of each input image to CV_64FC3
  // This will be allocated just the first time, since all your images have
  // the same size.
  //Mat temp;
  for (int i = 1; i < images.size(); ++i) {
    // Convert the input images to CV_64FC3 ...
    //RTCameraBaseLDBG_ << "converting "<<i;
    //cv::accumulate(*images[i], m);
    cv::addWeighted(m,0.5,*images[i],0.5,0,m);
    //images[i]->convertTo(temp, CV_64FC3);

    // ... so you can accumulate
    //m += temp;
  }
  // Convert back to CV_8UC3 type, applying the division to get the actual mean
   RTCameraBaseLDBG_ << "done "<<m.cols<<"x"<<m.rows;
  return m;
}
void RTCameraBase::updateProperty() {
  std::vector<std::string>          props;
  chaos::common::data::CDataWrapper camera_props;
  int                               ret = 0;
  if ((ret = driver->getCameraProperties(camera_props)) != 0) {
    RTCameraBaseLERR_ << "Error retriving camera properties ret:" << ret;
  } else {
    RTCameraBaseLDBG_ << "UpdateCamera from driver properties:" << camera_props.getJSONString();
  }
  camera_props.getAllKey(props);
  AttributeSharedCacheWrapper *cc = getAttributeCache();

  for (std::vector<std::string>::iterator i = props.begin(); i != props.end();
       i++) {
    if (!camera_props.isCDataWrapperValue(*i)) {
      if (camera_props.getValueType(*i) == chaos::DataType::TYPE_DOUBLE) {
        double tmp;
        tmp = camera_props.getDoubleValue(*i);

        // driver->getCameraProperty(*i, tmp);
        RTCameraBaseLDBG_ << "Camera Property double " << *i
                          << " VALUE:" << tmp;
        cc->setOutputAttributeValue(*i, tmp);
      }
      if (camera_props.getValueType(*i) == chaos::DataType::TYPE_INT32) {
        int32_t tmp, *p;
        // driver->getCameraProperty(*i, tmp);
        tmp = camera_props.getInt32Value(*i);
        RTCameraBaseLDBG_ << "Camera Property int " << *i << " VALUE:" << tmp;
#define WIDTH_KEY "WIDTH"
#define HEIGHT_KEY "HEIGHT"
#define OFFSETX_KEY "OFFSETX"
#define OFFSETY_KEY "OFFSETY"

        if (*i == WIDTH_KEY) {
          *osizex = tmp;
        }
        if (*i == HEIGHT_KEY) {
          *osizey = tmp;
        }
        if (*i == OFFSETX_KEY) {
          *ooffsetx = tmp;
        }
        if (*i == OFFSETY_KEY) {
          *ooffsety = tmp;
        }
        cc->setOutputAttributeValue(*i, tmp);
      }
      if (camera_props.getValueType(*i) == chaos::DataType::TYPE_STRING) {
        std::string tmp;
        tmp = camera_props.getStringValue(*i);

        RTCameraBaseLDBG_ << "Camera Property  " << *i << " VALUE:" << tmp;

        cc->setOutputAttributeValue(*i, tmp);
      }
    }
  }
}
#define SETINTPROP(name, n, value) \
  if (name == #n) {                \
    n = value;                     \
    return true;                   \
  }

bool RTCameraBase::setCamera(const std::string &name, int32_t value, uint32_t size) {
  /*if (name == "REFX") {
    *refx= value;

  } else if (name == "REFY") {
    *refy=value;
  } */
  return false;
}
void RTCameraBase::changeEncodingParam(int32_t val){
  encode_params.clear();
  if(*encoding==0){
    strcpy(encoding,".png");
    
  }
  if(!strcmp(encoding,".png")){
      encode_params.push_back(IMWRITE_PNG_COMPRESSION);
      encode_params.push_back(val);

      encode_params.push_back(IMWRITE_PNG_STRATEGY);
      encode_params.push_back(png_strategy);
  } else if(!strcmp(encoding,".jpg")){

      encode_params.push_back(IMWRITE_JPEG_QUALITY);
      encode_params.push_back(val);

  } else if(!strcmp(encoding,".webp")){
      encode_params.push_back(IMWRITE_WEBP_QUALITY);
      encode_params.push_back(val);

  } 
    compression_factor=val;

}

bool RTCameraBase::setCamera(const std::string &name, std::string value, uint32_t size) {
  if(name=="FMT"){
    if (value.size() == 0) {
      strcpy(encoding, ".png");

   } else {
    if(strcmp(encoding,value.c_str())&&(!strcmp(value.c_str(),"jpg"))){
      compression_factor=90;
    }
    if(strcmp(encoding,value.c_str())&&(!strcmp(value.c_str(),"webp"))){
      compression_factor=101; // looseless
    }
    snprintf(encoding, sizeof(encoding), ".%s", value.c_str());
    
  }
  // ofmt not .
  strncpy(ofmt, &encoding[1], sizeof(encoding));
  }
  changeEncodingParam(compression_factor);
  return true;
}

bool RTCameraBase::setCamera(const std::string &name, bool value, uint32_t size) {
  if (name == "ACQUIRE") {
    return setDrvProp(TRIGGER_MODE_KEY, (value == false) ? CAMERA_DISABLE_ACQUIRE : CAMERA_TRIGGER_CONTINOUS);
  } else if (name == "TRIGGER") {
    return setDrvProp(TRIGGER_MODE_KEY, (value == false) ? CAMERA_TRIGGER_CONTINOUS : CAMERA_TRIGGER_HW_HI);

  } else if (name == "PULSE") {
    /*return setDrvProp(TRIGGER_MODE_KEY, (value == false) ? CAMERA_TRIGGER_CONTINOUS
                                                      : CAMERA_TRIGGER_SINGLE);*/
    return setDrvProp(TRIGGER_MODE_KEY, (value == false) ? CAMERA_TRIGGER_CONTINOUS : CAMERA_TRIGGER_SINGLE);
  }
  return false;
}

bool RTCameraBase::setDrvProp(const std::string &name, int32_t value, uint32_t size) {
  int                          ret;
  int32_t                      valuer;
  AttributeSharedCacheWrapper *cc = getAttributeCache();
  /* if((name==OFFSETX_KEY) &&(*sizex!=*osizex)){
      RTCameraBaseLDBG_ << "CANNOT SET " << name << " BECAUSE WIDTH is out of set:" << *sizex;

    return true;
  }
  if((name==OFFSETY_KEY) &&(*sizey!=*osizey)){
      RTCameraBaseLDBG_ << "CANNOT SET " << name << " BECAUSE HEIGHT is out of set:" << *sizey;

    return true;
  }*/
  RTCameraBaseLDBG_ << "SETTING IPROP:" << name << " VALUE:" << value;
  bool stopgrab =
      (
       /* ((name == WIDTH_KEY) ) ||
       ((name == HEIGHT_KEY) ) ||
       ((name == OFFSETX_KEY) ) ||
       ((name == OFFSETY_KEY) ) ||*/
          ((name == TRIGGER_MODE_KEY)));

  // bool stopgrab = ((name == TRIGGER_MODE_KEY) && (value == CAMERA_DISABLE_ACQUIRE));
 /* if(((name == WIDTH_KEY) ) ||
       ((name == HEIGHT_KEY) ) ||
       ((name == OFFSETX_KEY) ) ||
       ((name == OFFSETY_KEY))){
         applyCalib=false;
       }*/
  if (stopgrab && (hasStopped() == false)) {
    stopGrabbing();
  }
  if ((name == TRIGGER_MODE_KEY) && (value == CAMERA_TRIGGER_SINGLE)) {
    *ppulse = true;

    // is equal to an HW trigger.
  }
  ret = driver->setCameraProperty(name, value);
  setStateVariableSeverity(StateVariableTypeAlarmDEV, "error_setting_property", chaos::common::alarm::MultiSeverityAlarmLevelClear);

  if (ret != 0) {
    std::stringstream ss;
    ss << "error setting \"" << name << "\" to " << value << " ret:" << ret;
    setStateVariableSeverity(
        StateVariableTypeAlarmDEV, "error_setting_property", chaos::common::alarm::MultiSeverityAlarmLevelWarning);
    /* metadataLogging(
         chaos::common::metadata_logging::StandardLoggingChannel::LogLevelError,
         ss.str());*/
  }
  int32_t *pmode = cc->getRWPtr<int32_t>(DOMAIN_OUTPUT, name);

  if ((name == TRIGGER_MODE_KEY)) {
    // updateProperty();

    switch (mode) {
      case CAMERA_DISABLE_ACQUIRE:
        getAttributeCache()->setInputAttributeValue("ACQUIRE", false);
        getAttributeCache()->setInputAttributeValue("TRIGGER", false);
        getAttributeCache()->setInputAttributeValue("PULSE", false);
        break;
      case CAMERA_TRIGGER_CONTINOUS:
        getAttributeCache()->setInputAttributeValue("ACQUIRE", true);
        getAttributeCache()->setInputAttributeValue("TRIGGER", false);
        getAttributeCache()->setInputAttributeValue("PULSE", false);
        break;
      case CAMERA_TRIGGER_HW_HI:
      case CAMERA_TRIGGER_HW_LOW:
        getAttributeCache()->setInputAttributeValue("ACQUIRE", true);
        getAttributeCache()->setInputAttributeValue("TRIGGER", true);
        getAttributeCache()->setInputAttributeValue("PULSE", false);
        break;
      case CAMERA_TRIGGER_SINGLE:
      case CAMERA_TRIGGER_SOFT:
        getAttributeCache()->setInputAttributeValue("ACQUIRE", true);
        getAttributeCache()->setInputAttributeValue("TRIGGER", false);
        getAttributeCache()->setInputAttributeValue("PULSE", true);
        break;
    }
  }
  RTCameraBaseLDBG_ << "SET IPROP:" << name << " SET VALUE:" << value << " ret:" << ret << " mode:" << mode;
  if (hasStopped() == false) {
    if (stopgrab &&
        !((name == TRIGGER_MODE_KEY) && (value == CAMERA_DISABLE_ACQUIRE))) {
      startGrabbing();
    } else if (stopCapture && ((name == TRIGGER_MODE_KEY) &&
                               (value != CAMERA_DISABLE_ACQUIRE))) {
      startGrabbing();
    }
  }
  updateDatasetFromDriverProperty();
  if ((name == TRIGGER_MODE_KEY)) {
    driver->getCameraProperty(name, valuer);

    mode = *pmode = ((value != CAMERA_TRIGGER_SINGLE) ? valuer : CAMERA_TRIGGER_SINGLE);
  }

  pushOutputDataset();
  return (ret == 0);
}

bool RTCameraBase::setDrvProp(const std::string &name, double value, uint32_t size) {
  int    ret;
  double valuer;
  RTCameraBaseLDBG_ << "SET FPROP:" << name << " VALUE:" << value << endl;
  // SETINTPROP(name,ZOOMX,value);
  // SETINTPROP(name,ZOOMY,value);

  ret = driver->setCameraProperty(name, value);
  setStateVariableSeverity(StateVariableTypeAlarmDEV, "error_setting_property", chaos::common::alarm::MultiSeverityAlarmLevelClear);

  if (ret != 0) {
    setStateVariableSeverity(
        StateVariableTypeAlarmDEV, "error_setting_property", chaos::common::alarm::MultiSeverityAlarmLevelWarning);
  }
  // updateProperty();
  driver->getCameraProperty(name, valuer);
  AttributeSharedCacheWrapper *cc = getAttributeCache();
  double                      *p  = cc->getRWPtr<double>(DOMAIN_OUTPUT, name);
  *p                              = valuer;
  getAttributeCache()->setInputDomainAsChanged();
  getAttributeCache()->setOutputDomainAsChanged();

  return (ret == 0);
}
//! Return the definition of the control unit
/*!
The api that can be called withi this method are listed into
"Control Unit Definition Public API" module into html documentation
(chaosframework/Documentation/html/group___control___unit___definition___api.html)
*/
void RTCameraBase::unitDefineActionAndDataset() throw(chaos::CException) {
  chaos::common::data::CDataWrapper camera_props;
  if (driver == NULL) {
    chaos::cu::driver_manager::driver::DriverAccessor *acc =
        getAccessoInstanceByIndex(0);
    if (acc == NULL) {
      throw chaos::CException(
          -1, "No associated camera driver for:" + getDeviceID(), __PRETTY_FUNCTION__);
    }
    driver = new CameraDriverInterface(acc);
    if (driver == NULL) {
      throw chaos::CException(-1, "cannot create driver for:" + getDeviceID(), __PRETTY_FUNCTION__);
    }
  }
  useCustomHigResolutionTimestamp(true);

  addAttributeToDataSet("ROT", "Rotation angle (degree)", chaos::DataType::TYPE_INT32, chaos::DataType::Input);
  addAttributeToDataSet("REFABS", "Absolute(true)/Relative(false)", chaos::DataType::TYPE_BOOLEAN, chaos::DataType::Input);

  addAttributeToDataSet("REFX", "Reference centerX", chaos::DataType::TYPE_INT32, chaos::DataType::Input);
  addAttributeToDataSet("REFY", "Reference centerY", chaos::DataType::TYPE_INT32, chaos::DataType::Input);

  addAttributeToDataSet("REFSX", "Reference centerSX", chaos::DataType::TYPE_INT32, chaos::DataType::Input);
  addAttributeToDataSet("REFSY", "Reference centerSY", chaos::DataType::TYPE_INT32, chaos::DataType::Input);
  addAttributeToDataSet("REFRHO", "Reference centerRho", chaos::DataType::TYPE_DOUBLE, chaos::DataType::Input);

  /****
   *
   */
  addAttributeToDataSet("FMT", "image format (jpg,png,gif...)", chaos::DataType::TYPE_STRING, chaos::DataType::Bidirectional, 16);
  if (buffering > 0) {
    addAttributeToDataSet("CAPTURE_FRAMERATE", "Capture Frame Rate", chaos::DataType::TYPE_INT32, chaos::DataType::Output);
    addAttributeToDataSet("ENCODE_FRAMERATE", "Encode Frame Rate", chaos::DataType::TYPE_INT32, chaos::DataType::Output);
  }
  addAttributeToDataSet("ACQUIRE", "Is Acquiring", chaos::DataType::TYPE_BOOLEAN, chaos::DataType::Bidirectional);
  addAttributeToDataSet("TRIGGER", "Is triggered ", chaos::DataType::TYPE_BOOLEAN, chaos::DataType::Bidirectional);
  addAttributeToDataSet("PULSE", "Is pulse", chaos::DataType::TYPE_BOOLEAN, chaos::DataType::Bidirectional);
  addHandlerOnInputAttributeName< ::driver::sensor::camera::RTCameraBase, bool>(
      this, &::driver::sensor::camera::RTCameraBase::setCamera, "ACQUIRE");
  addHandlerOnInputAttributeName< ::driver::sensor::camera::RTCameraBase, std::string>(
      this, &::driver::sensor::camera::RTCameraBase::setCamera, "FMT");
  addHandlerOnInputAttributeName< ::driver::sensor::camera::RTCameraBase, bool>(
      this, &::driver::sensor::camera::RTCameraBase::setCamera, "TRIGGER");
  addHandlerOnInputAttributeName< ::driver::sensor::camera::RTCameraBase, bool>(
      this, &::driver::sensor::camera::RTCameraBase::setCamera, "PULSE");
  addBinaryAttributeAsMIMETypeToDataSet("FRAMEBUFFER", "output image", "image/png", chaos::DataType::Output);
  /*
 if (driver->getCameraProperties(camera_props) != 0) {
    throw chaos::CException(-1, "Error retrieving camera properties:"+driver->getLastError(),
                            __PRETTY_FUNCTION__);
  }
  std::vector<std::string> props;
  camera_props.getAllKey(props);

  for (std::vector<std::string>::iterator i = props.begin(); i != props.end();
       i++) {
    if (!camera_props.isCDataWrapperValue(*i)) {
      RTCameraBaseLDBG_ << "ADDING ATTRIBUTE:" << *i
                        << " Type:" << camera_props.getValueType(*i);

      addAttributeToDataSet(*i, *i, camera_props.getValueType(*i),
                            chaos::DataType::Bidirectional);
      // getAttributeCache()->addCustomAttribute(*i, 8,
      // camera_props.getValueType(*i));
      // getAttributeCache()->setCustomAttributeValue("quit", &quit,
      // sizeof(bool));
      // addCustomAttribute(*i,sizeof(double),camera_props.getValueType(*i));
      if (camera_props.getValueType(*i) == chaos::DataType::TYPE_DOUBLE) {
        addHandlerOnInputAttributeName< ::driver::sensor::camera::RTCameraBase,
                                       double>(
            this, &::driver::sensor::camera::RTCameraBase::setDrvProp, *i);
      }
      if (camera_props.getValueType(*i) == chaos::DataType::TYPE_INT32) {
        addHandlerOnInputAttributeName< ::driver::sensor::camera::RTCameraBase,
                                       int32_t>(
            this, &::driver::sensor::camera::RTCameraBase::setDrvProp, *i);
      }
    }
  }
*/

  addPublicDriverPropertyToDataset();

  /*addBinaryAttributeAsSubtypeToDataSet(
      "FRAMEBUFFER", "image", chaos::DataType::SUB_TYPE_CHAR,
      DEFAULT_RESOLUTION, chaos::DataType::Output);*/
  addStateVariable(
      StateVariableTypeAlarmDEV, "error_setting_property", "the operation in not supported i.e property not present or accessible");
  addStateVariable(StateVariableTypeAlarmCU, "encode_error", "an error occurred during encode", LOG_FREQUENCY);
  addStateVariable(StateVariableTypeAlarmCU, "calibration_error", "an error during calibration (i.e. different size) ", LOG_FREQUENCY);

  addStateVariable(StateVariableTypeAlarmCU, "auto_reference_error", "an error occurred during autoreference", LOG_FREQUENCY);
  addStateVariable(StateVariableTypeAlarmDEV, "camera_bandwidth", "camera bandwith", LOG_FREQUENCY);

  addStateVariable(StateVariableTypeAlarmDEV, "capture_error", "an error occurred during capture", LOG_FREQUENCY);
  addStateVariable(StateVariableTypeAlarmDEV, "capture_timeout", "a timeout has occurred", LOG_FREQUENCY);
  addStateVariable(StateVariableTypeAlarmDEV, "camera_disconnect", "The camera appears disconnected", LOG_FREQUENCY);

  addStateVariable(StateVariableTypeAlarmCU, "captureQueue", "Queue Capture warning half, full error", LOG_FREQUENCY);
  addStateVariable(StateVariableTypeAlarmCU, "encodeQueue", "Queue Encode warning half, full error", LOG_FREQUENCY);
}

//! Define custom control unit attribute
void RTCameraBase::unitDefineCustomAttribute() {
  /*std::string config;
  chaos::common::data::CDataWrapper attr;

  if (driver->getCameraProperties(attr) != 0) {
    throw chaos::CException(-1, "Error retrieving camera properties",
                            __PRETTY_FUNCTION__);
  }
  config = attr.getCompliantJSONString();
 */
  // RTCameraBaseLDBG_ << "ADDING CONFIG:" << config;
  // setDriverInfo(attr);
  // getAttributeCache()->setCustomAttributeValue(chaos::ControlUnitNodeDefinitionKey::CONTROL_UNIT_DRIVER_INFO,
  // (void *)config.c_str(),
  //    config.size()+1);
  createProperty(
      "cameraStreamEnable", cameraStreamEnable, "cameraStreamEnable", [](AbstractControlUnit *thi, const std::string &name, const chaos::common::data::CDataWrapper &p) -> chaos::common::data::CDWUniquePtr {
        chaos::common::data::CDWUniquePtr ret(new chaos::common::data::CDataWrapper());
        ret->addBoolValue(PROPERTY_VALUE_KEY,((RTCameraBase*)thi)->cameraStreamEnable);
        return ret; }, [](AbstractControlUnit *thi, const std::string &name, const chaos::common::data::CDataWrapper &p) -> chaos::common::data::CDWUniquePtr { 
          ((RTCameraBase*)thi)->cameraStreamEnable=p.getBoolValue(PROPERTY_VALUE_KEY);
          ((RTCameraBase*)thi)->updateStreamLink();
        
          return p.clone(); });
  if(streamer&&cameraStreamEnable){
    updateStreamLink();
    
  } 

}
void RTCameraBase::updateStreamLink(){
    getAttributeCache()->addCustomAttribute("stream", 128, chaos::DataType::TYPE_STRING);


    if(streamer&&cameraStreamEnable){

      std::string streampath="http://"+HttpStreamManager::getInstance()->getRoot()+streamName;
      getAttributeCache()->setCustomAttributeValue("stream",(void*)streampath.c_str(),streampath.length());
      getAttributeCache()->setCustomDomainAsChanged();
      pushCustomDataset();
    } else {
      getAttributeCache()->setCustomAttributeValue("stream",(void*)"",1);
      getAttributeCache()->setCustomDomainAsChanged();
      pushCustomDataset();

    }
  

}

//! Initialize the Custom Control Unit
void RTCameraBase::unitInit() throw(chaos::CException) {
  int     ret;
  int32_t itype;
  int32_t width, height;

  AttributeSharedCacheWrapper *cc = getAttributeCache();
  performCalib                    = false;
  // this properties must exist
  //
  sizex  = cc->getRWPtr<int32_t>(DOMAIN_INPUT, WIDTH_KEY);
  sizey  = cc->getRWPtr<int32_t>(DOMAIN_INPUT, HEIGHT_KEY);
  osizex = cc->getRWPtr<int32_t>(DOMAIN_OUTPUT, WIDTH_KEY);
  osizey = cc->getRWPtr<int32_t>(DOMAIN_OUTPUT, HEIGHT_KEY);

  offsetx  = cc->getRWPtr<int32_t>(DOMAIN_INPUT, OFFSETX_KEY);
  offsety  = cc->getRWPtr<int32_t>(DOMAIN_INPUT, OFFSETY_KEY);
  ooffsetx = cc->getRWPtr<int32_t>(DOMAIN_OUTPUT, OFFSETX_KEY);
  ooffsety = cc->getRWPtr<int32_t>(DOMAIN_OUTPUT, OFFSETY_KEY);

  //
  rot = cc->getROPtr<int32_t>(DOMAIN_INPUT, "ROT");

  mode  = *cc->getROPtr<int32_t>(DOMAIN_INPUT, TRIGGER_MODE_KEY);
  omode = cc->getRWPtr<int32_t>(DOMAIN_OUTPUT, TRIGGER_MODE_KEY);
  //   camera_out=cc->getRWPtr<uint8_t>(DOMAIN_OUTPUT, "FRAMEBUFFER");
  fmt  = cc->getRWPtr<char>(DOMAIN_INPUT, "FMT");
  ofmt = cc->getRWPtr<char>(DOMAIN_OUTPUT, "FMT");

  refx   = cc->getRWPtr<int32_t>(DOMAIN_INPUT, "REFX");
  refy   = cc->getRWPtr<int32_t>(DOMAIN_INPUT, "REFY");
  refsx  = cc->getRWPtr<int32_t>(DOMAIN_INPUT, "REFSX");
  refsy  = cc->getRWPtr<int32_t>(DOMAIN_INPUT, "REFSY");
  refabs = cc->getRWPtr<bool>(DOMAIN_INPUT, "REFABS");
  refrho = cc->getRWPtr<double>(DOMAIN_INPUT, "REFRHO");

  pacquire  = cc->getRWPtr<bool>(DOMAIN_OUTPUT, "ACQUIRE");
  ptrigger  = cc->getRWPtr<bool>(DOMAIN_OUTPUT, "TRIGGER");
  ppulse    = cc->getRWPtr<bool>(DOMAIN_OUTPUT, "PULSE");
  *pacquire = false;
  *ptrigger = false;
  *ppulse   = false;

  chaos::common::data::CDWUniquePtr calibrazione = loadData("calibration_image");
  if (applyCalib&& calibrazione.get()) {
      uint32_t    size;
      const char *buf = calibrazione->getBinaryValue("FRAMEBUFFER", size);
      if (buf && size) {
        std::vector<uchar> data = std::vector<uchar>(buf, buf + size);
        cv::Mat            sub  = cv::imdecode(data, IMREAD_ANYCOLOR | IMREAD_ANYDEPTH);
        if (sub.data) {
          if (subImage) {
            delete subImage;
          }
          subImage = new cv::Mat(sub);
          RTCameraBaseLDBG_ << " Loading CALIB DATA:" << size << " bytes";
        } else {
          RTCameraBaseLERR_ << " Invalid calibration data " << size << " bytes";
        }
      }
    }
  if (buffering > 0) {
    capture_frame_rate =
        cc->getRWPtr<int32_t>(DOMAIN_OUTPUT, "CAPTURE_FRAMERATE");
    enc_frame_rate = cc->getRWPtr<int32_t>(DOMAIN_OUTPUT, "ENCODE_FRAMERATE");
  }

  // breanch number and soft reset

  if ((ret = driver->cameraInit(0, 0)) != 0) {
    throw chaos::CException(ret, "cannot initialize camera:" + driver->getLastError(), __PRETTY_FUNCTION__);
  }

  chaos::common::data::CDataWrapper cam_prop;
  if (driver->getCameraProperties(cam_prop) == 0) {
    if (cam_prop.hasKey(FRAMEBUFFER_ENCODING_KEY) &&
        cam_prop.isStringValue(FRAMEBUFFER_ENCODING_KEY)) {
      framebuf_encoding_s = cam_prop.getStringValue(FRAMEBUFFER_ENCODING_KEY);
      framebuf_encoding   = fmt2cv(framebuf_encoding_s);
      std::string enc;
      bpp = cv2fmt(framebuf_encoding, enc);
    }
    if (cam_prop.hasKey(OFFSETX_KEY) && cam_prop.isInt32Value(OFFSETX_KEY)) {
      *ooffsetx = cam_prop.getInt32Value(OFFSETX_KEY);
    }
    if (cam_prop.hasKey(OFFSETY_KEY) && cam_prop.isInt32Value(OFFSETY_KEY)) {
      *ooffsety = cam_prop.getInt32Value(OFFSETY_KEY);
    }
    if (cam_prop.hasKey(WIDTH_KEY) && cam_prop.isInt32Value(WIDTH_KEY)) {
      *osizex = cam_prop.getInt32Value(WIDTH_KEY);
    }
    if (cam_prop.hasKey(HEIGHT_KEY) && cam_prop.isInt32Value(HEIGHT_KEY)) {
      *osizey = cam_prop.getInt32Value(HEIGHT_KEY);
    }
  }
  if (*sizex == 0) {
    *sizex = *osizex;
    RTCameraBaseLERR_ << "INVALID SET SIZEX 0 force to:" << *osizex;
  }
  if (*sizey == 0) {
    *sizey = *osizey;
    RTCameraBaseLDBG_ << "INVALID SET SIZEY 0 force to:" << *osizey;
  }

  /*if ((*sizex == 0) || (*sizey == 0)) {
    std::stringstream ss;
    ss << "Invalid image sizes:" << *sizex << "x" << *sizey;

    throw chaos::CException(-3, ss.str(), __PRETTY_FUNCTION__);
  }*/
  if (!apply_resize) {
    imagesizex = *sizex;
    imagesizey = *sizey;
  }
  //cc->setOutputAttributeNewSize("FRAMEBUFFER", imagesizex * imagesizey * (bpp), true);

  // framebuf = (uint8_t*)malloc(size);
  bpp = cv2fmt(framebuf_encoding, framebuf_encoding_s);

  RTCameraBaseLDBG_ << "Going to acquire images " << *sizex << "x" << *sizey
                    << " bpp:" << bpp
                    << " framebuf_encoding:" << framebuf_encoding_s << " ("
                    << framebuf_encoding << ") ";
  if (*fmt == 0) {
    strcpy(encoding, ".png");

  } else {
    snprintf(encoding, sizeof(encoding), ".%s", fmt);
  }
  strncpy(ofmt, &encoding[1], sizeof(encoding));
  changeEncodingParam(compression_factor);
  updateProperty();
}
static void* encode(void*ptr){
  RTCameraBase*p=(RTCameraBase*)ptr;

  p->encodeThread(p->encode_id++);
  return NULL;

}
static void* capture(void*ptr){
  RTCameraBase*p=(RTCameraBase*)ptr;
  
  p->captureThread();
  return NULL;
}
void RTCameraBase::cameraGrabCallBack(const void *buf, uint32_t blen, uint32_t width, uint32_t heigth, uint32_t error) {}
void RTCameraBase::startGrabbing() {
  RTCameraBaseLDBG_ << "Start Grabbing";
  metadataLogging(
      chaos::common::metadata_logging::StandardLoggingChannel::LogLevelInfo,
      "Start grabbing");

  // allocate buffers;

  /*   for(int cnt=0;cnt<CAMERA_FRAME_BUFFERING;cnt++){
        framebuf_out[cnt].buf=(unsigned char*)malloc(*sizex * *sizey * 4);
        framebuf_out[cnt].size=*sizex * *sizey * 4;
    }*/

  updateProperty();
  RTCameraBaseLDBG_ << "Buffering:" << buffering << " buffers of "
                    << (*sizex * *sizey * bpp) << " bytes";
#ifdef CAMERA_ENABLE_SHARED
  try {
    if (shared_mem.get() == NULL) {
      shared_mem.reset(new ::common::misc::data::SharedMem(
          getCUID(), (imagesizex * imagesizey * bpp)));
      RTCameraBaseLDBG_ << "opened sharedMem \"" << shared_mem->getName()
                        << "\" at @" << std::hex << shared_mem->getMem()
                        << " size:" << std::dec << shared_mem->getSize();
    } else {
      shared_mem->resize(imagesizex * imagesizey * bpp);
    }
  } catch (boost::interprocess::interprocess_exception &ex) {
    RTCameraBaseLERR_ << "##cannot open shared memory :" << ex.what();
  }
#endif
  getAttributeCache()->setCustomDomainAsChanged();
  pushCustomDataset();
  driver->startGrab(0);
  if (stopCapture == true) {
    stopCapture = false;
    if (buffering > 0) {
      pthread_attr_t thread_attr;
      struct     sched_param  param;
      encode_id=0;
      pthread_attr_init(&thread_attr);  // Initialise the attributes
      pthread_attr_setschedpolicy(&thread_attr, SCHED_RR);  // Set attributes to RR p
      param.sched_priority = 50;
      pthread_attr_setschedparam(&thread_attr, &param); // Set attributes to priority 30
      pthread_create(&capture_th, &thread_attr, capture, (void*)this);
      char name[256];
      sprintf(name,"capture_th_%s",getDeviceID().c_str());
      pthread_setname_np(capture_th, name);

      param.sched_priority = 99;
      pthread_attr_setschedpolicy(&thread_attr, SCHED_FIFO);  // must be yield by processor, no timeslice
      for(int cnt=0;cnt<encode_thds;cnt++){
        pthread_create(&encode_th[cnt], &thread_attr, encode, (void*)this);
        sprintf(name,"encode_th%d_%s",cnt,getDeviceID().c_str());

        pthread_setname_np(encode_th[cnt], name);


      }
      pthread_attr_destroy(&thread_attr); // We've done with the attributes
      


    //  capture_th = std::thread(&RTCameraBase::captureThread, this);
    //  encode_th = std::thread(&RTCameraBase::encodeThread, this);

    }
  }
}
void RTCameraBase::haltThreads() {
  int ret;
  if(stopCapture ){
    return;
  }
  RTCameraBaseLDBG_ << "Stop Grab low level:" << stopCapture<<" capture exited:"<<capture_exited<<" buffering:"<<buffering;

  stopCapture = true;
  if (buffering > 0) {
   
    do {
        for(int cnt=0;cnt<encode_thds;cnt++){
            captureImg[cnt].unblock();
            encodedImg[cnt].unblock();
      }
      encoded_queue.unblock();
      RTCameraBaseLDBG_ << "unblocking fifo ";
    } while(capture_exited==false);
    
     if(pthread_self()!=capture_th){
      //  pthread_cancel(capture_th);
        RTCameraBaseLDBG_ << "Joining capture thread pid:"<<std::hex<<capture_th;

        pthread_join(capture_th,NULL);
     //   RTCameraBaseLDBG_ << "Joining capture thread OK";

    }
    for(int cnt=0;cnt<encode_thds;cnt++){

    if(pthread_self()!=encode_th[cnt]){
      RTCameraBaseLDBG_ << "Joining encode thread "<<cnt<<" pid:"<<std::hex<<encode_th[cnt];

     // pthread_cancel(encode_th[cnt]);
      pthread_join(encode_th[cnt],NULL);
     // RTCameraBaseLDBG_ << "Joining encode thread OK";

    }
    }
    

#if 0
    if (encode_th.joinable()) {
      if (std::this_thread::get_id() != encode_th.get_id()) {
          encode_th.join();
          RTCameraBaseLDBG_ << "encodeThread joined!";
      }
    }
    if (capture_th.joinable()) {
      if (std::this_thread::get_id() != capture_th.get_id()) {
        capture_th.join();

      }
    }
#endif
  }

  RTCameraBaseLDBG_ << "Stopped Grab driver:" << ret;
}

void RTCameraBase::stopGrabbing() {
  RTCameraBaseLDBG_ << "Stop Grabbing..." << stopCapture;
  haltThreads();
  
    
  /* metadataLogging(
        chaos::common::metadata_logging::StandardLoggingChannel::LogLevelInfo,
        "Stop grabbing");
*/
  driver->stopGrab();

  RTCameraBaseLDBG_ << "Stop Grabbing done:" << stopCapture;

  /*    for(int cnt=0;cnt<CAMERA_FRAME_BUFFERING;cnt++){
        free(framebuf_out[cnt].buf);
        framebuf_out[cnt].size=0;
        framebuf_out[cnt].buf=NULL;
    }*/
  for(int cnt=0;cnt<encode_thds;cnt++){
    captureImg[cnt].get().consume_all([this,cnt](camera_buf_t *i) {
    RTCameraBaseLDBG_ << cnt<< "] Capture frames deleted :" << static_cast<void*>(i);

    delete (i);
   
  });
    encodedImg[cnt].get().consume_all([this,cnt](Encoder* i) {
     RTCameraBaseLDBG_ << cnt<< "] Encode frames deleted :" << static_cast<void*>(i);

    delete (i);
  });
   encoded_queue.get().consume_all([](int i){

   });
  }
  setStateVariableSeverity(StateVariableTypeAlarmDEV,chaos::common::alarm::MultiSeverityAlarmLevelClear);
  setStateVariableSeverity(StateVariableTypeAlarmCU,chaos::common::alarm::MultiSeverityAlarmLevelClear);
}

//! Execute the work, this is called with a determinated delay, it must be as
//! fast as possible
void RTCameraBase::unitStart() throw(chaos::CException) {
  RTCameraBaseLDBG_ << "Start - stopped flag:" << hasStopped();
  getAttributeCache()->setInputDomainAsChanged();
  getAttributeCache()->setOutputDomainAsChanged();
  getAttributeCache()->setCustomDomainAsChanged();
  performAutoReference = false;
  pushCustomDataset();
  startGrabbing();
  setStateVariableSeverity(StateVariableTypeAlarmDEV,chaos::common::alarm::MultiSeverityAlarmLevelClear);
  setStateVariableSeverity(StateVariableTypeAlarmCU,chaos::common::alarm::MultiSeverityAlarmLevelClear);
}

void RTCameraBase::captureThread() {
  int           ret;
  camera_buf_t *img = 0;
  uint64_t      start;
  counter_capture = 0;
  RTCameraBaseLDBG_ << "Capture thread STARTED";
  
  capture_cnt=0;
  capture_exited=false;
  while ((!stopCapture) && (hasStopped() == false)) {
    img   = 0;
    start = chaos::common::utility::TimingUtil::getTimeStampInMicroseconds();

    ret = driver->waitGrab(&img, trigger_timeout);

    if ((img != 0) && (ret > 0)) {
      // keep alarm high
      setStateVariableSeverity(
          StateVariableTypeAlarmDEV, "capture_error", chaos::common::alarm::MultiSeverityAlarmLevelClear);
      setStateVariableSeverity(
          StateVariableTypeAlarmDEV, "capture_timeout", chaos::common::alarm::MultiSeverityAlarmLevelClear);
      setStateVariableSeverity(StateVariableTypeAlarmDEV, "camera_bandwidth", chaos::common::alarm::MultiSeverityAlarmLevelClear);

     setStateVariableSeverity(
          StateVariableTypeAlarmDEV, "camera_disconnect", chaos::common::alarm::MultiSeverityAlarmLevelClear);
      int pushret;
      int  retry = 3;
      if((captureImg[capture_cnt].length())&&(captureImg[capture_cnt].length()<CAMERA_FRAME_BUFFERING-1)){
       // boost::this_thread::yield();
        setStateVariableSeverity(
                StateVariableTypeAlarmCU, "captureQueue", chaos::common::alarm::MultiSeverityAlarmLevelWarning);
        RTCameraBaseLERR_ << "Warning Capture queue "<<capture_cnt<<":" << captureImg[capture_cnt].length();


      } else if(captureImg[capture_cnt].length()==CAMERA_FRAME_BUFFERING){
        RTCameraBaseLERR_ << "Error Capture queue "<<capture_cnt<<" :" << captureImg[capture_cnt].length();
      //  boost::this_thread::yield();

setStateVariableSeverity(
                StateVariableTypeAlarmCU, "captureQueue", chaos::common::alarm::MultiSeverityAlarmLevelHigh);
        if(stopCapture){
          continue;
        }
      } else {
          setStateVariableSeverity(
StateVariableTypeAlarmCU, "captureQueue", chaos::common::alarm::MultiSeverityAlarmLevelClear);

      }
      
      pushret = captureImg[capture_cnt].push(img);

      if (pushret < 0) {
        RTCameraBaseLERR_ << "Error on Queue:" << pushret << " queue:" << captureImg[0].length();
        setStateVariableSeverity(
                StateVariableTypeAlarmCU, "captureQueue", chaos::common::alarm::MultiSeverityAlarmLevelHigh);
        continue;
      }
      if(encode_thds>1){
            encoded_queue.push(capture_cnt);
      }
      capture_cnt=((capture_cnt+1)%encode_thds);
      capture_time +=
          (chaos::common::utility::TimingUtil::getTimeStampInMicroseconds() -
           start);
      counter_capture++;

    } else {
      if (stopCapture == false) {
        if ((ret == chaos::ErrorCode::EC_GENERIC_TIMEOUT)) {
          if (mode != CAMERA_TRIGGER_SINGLE) {
            RTCameraBaseLERR_ << " wait returned:" << ret
                              << " timeout stopped:" << stopCapture;
            setStateVariableSeverity(
                StateVariableTypeAlarmDEV, "capture_timeout", chaos::common::alarm::MultiSeverityAlarmLevelWarning);
          }
          continue;
        } 
        switch(ret){
        case  chaos::ErrorCode::EC_GENERIC_TIMEOUT:
        RTCameraBaseLERR_ << " TIMEOUT:" << ret;
        setStateVariableSeverity(
            StateVariableTypeAlarmDEV, "capture_timeout", chaos::common::alarm::MultiSeverityAlarmLevelWarning);

        break;
        case CAMERA_DRIVER_DISCONNECT:
            RTCameraBaseLERR_ << " DISCONNECTED:" << ret;

        setStateVariableSeverity(
          StateVariableTypeAlarmDEV, "camera_disconnect", chaos::common::alarm::MultiSeverityAlarmLevelHigh);
        break;
        case CAMERA_BANDWIDTH_ERROR:
        RTCameraBaseLERR_ << " BANDWITH :" << ret;

        setStateVariableSeverity(
          StateVariableTypeAlarmDEV, "camera_bandwidth", chaos::common::alarm::MultiSeverityAlarmLevelHigh);
        break;
        default:
          RTCameraBaseLERR_ << " Error wait returned:" << ret;
        setStateVariableSeverity(
            StateVariableTypeAlarmDEV, "capture_error", chaos::common::alarm::MultiSeverityAlarmLevelHigh);
      } 
        
      }
    }
  }
  RTCameraBaseLDBG_ << "Capture thread ENDED";
  for(int cnt=0;cnt<encode_thds;cnt++){
    RTCameraBaseLDBG_ << "Queue " <<cnt<<":"<< captureImg[cnt].length() << " has Stopped:" << hasStopped();
  }
  capture_exited=true;

}
Mat rotate(Mat src, int angle) {
  Mat dst;
  if (angle == -90) {
    cv::rotate(src, dst, cv::ROTATE_90_CLOCKWISE);

  } else if (angle == 90) {
    cv::rotate(src, dst, cv::ROTATE_90_COUNTERCLOCKWISE);

  } else if (angle == 180) {
    cv::rotate(src, dst, cv::ROTATE_180);

  } else if (angle == -270) {
    cv::rotate(src, dst, cv::ROTATE_90_CLOCKWISE);

  } else if (angle == 270) {
    cv::rotate(src, dst, cv::ROTATE_90_COUNTERCLOCKWISE);

  } else {
    Point2f    pt(src.cols / 2., src.rows / 2.);
    Mat        r    = getRotationMatrix2D(pt, angle, 1.0);
    cv::Rect2f bbox = cv::RotatedRect(cv::Point2f(), src.size(), angle).boundingRect2f();
    // adjust transformation matrix
    r.at<double>(0, 2) += bbox.width / 2.0 - src.cols / 2.0;
    r.at<double>(1, 2) += bbox.height / 2.0 - src.rows / 2.0;
    warpAffine(src, dst, r, Size(src.cols, src.rows));
  }
  return dst;
}
void RTCameraBase::encodeThread(int indx) {
  uint64_t start;
  // bpp = cv2fmt(framebuf_encoding, framebuf_encoding_s);
  setStateVariableSeverity(StateVariableTypeAlarmCU, "encode_error", chaos::common::alarm::MultiSeverityAlarmLevelClear);

  RTCameraBaseLDBG_ <<indx<<"] Encode " << encoding
                    << " , assuming framebuf:" << framebuf_encoding_s << "("
                    << framebuf_encoding << ") bpp:" << bpp
                    << " thread STARTED "<<CV_8UC1;
  std::string fname = getDeviceID();
  replace(fname.begin(), fname.end(), '/', '_');
#ifdef CAMERA_GEN_VIDEO

  VideoWriter video(fname + ".avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 10, Size(*sizex, *sizey));
#endif
  while ((!stopCapture) && (hasStopped() == false)) {
    camera_buf_t *framebuf;
    // RTCameraBaseLDBG_ << "popping image queue:"<<captureImg[0].length();
    int ret = captureImg[indx].wait_and_pop(framebuf);
    if ((ret >= 0) && framebuf && framebuf->buf) {
      start = chaos::common::utility::TimingUtil::getTimeStampInMicroseconds();
      int isizex = framebuf->width;
      int isizey = framebuf->height;

      if (isizey * isizex * bpp > framebuf->size) {
        setStateVariableSeverity(StateVariableTypeAlarmCU, "encode_error", chaos::common::alarm::MultiSeverityAlarmLevelHigh);
        continue;
      }

      cv::Mat image;
      if(applyCalib && subImage){
        if((subImage->cols !=isizex) &&(subImage->rows != isizey)){
          RTCameraBaseLDBG_ << "Disabling calibration size mismatch calib:"<<subImage->rows<<"x"<<subImage->cols<<" image:"<<isizex<<isizey;
          setStateVariableSeverity(StateVariableTypeAlarmCU, "calibration_error", chaos::common::alarm::MultiSeverityAlarmLevelHigh);

          delete subImage;
          subImage=NULL;
          applyCalib=false;
        } else {
            setStateVariableSeverity(StateVariableTypeAlarmCU, "calibration_error", chaos::common::alarm::MultiSeverityAlarmLevelClear);

        }
             
      }
      try {
        if (framebuf_encoding == cv::COLOR_BayerBG2RGB) {
          //   cv::Mat mat16uc1(*sizey, *sizex, CV_16UC1, a.buf);
          cv::Mat mat8uc1(isizey, isizex, CV_8UC1, framebuf->buf);
          // cv::Mat mat16uc3_rgb(*sizey, *sizex, CV_16UC3);
          // cv::cvtColor(mat16uc1, mat16uc3_rgb, cv::COLOR_BayerBG2RGB);
          cv::Mat mat8uc3_rgb(isizey, isizex, CV_8UC3);
          cv::cvtColor(mat8uc1, mat8uc3_rgb, cv::COLOR_BayerBG2RGB);

          // mat16uc3_rgb.convertTo(mat8uc3_rgb, CV_8UC3);
          if (applyCalib && subImage) {
           
            
            image = (mat8uc3_rgb - *subImage);

          } else {
            image = mat8uc3_rgb;
          }

        } else if (framebuf_encoding == cv::COLOR_YUV2RGB_NV21) {
          cv::Mat mat16uc1 = cv::Mat(isizey, isizex, CV_8UC2, framebuf->buf);
          // cv::Mat mat16uc3_rgb(*sizey, *sizex, CV_16UC3);

          cv::Mat mat8uc3_rgb(isizey, isizex, CV_8UC3);
          // cv::cvtColor(mat16uc1, mat8uc3_rgb, cv::COLOR_YUV2RGB );//
          cv::cvtColor(mat16uc1, mat8uc3_rgb, cv::COLOR_YUV2RGB_UYVY);  //

          if (applyCalib && subImage) {
            image = (mat8uc3_rgb - *subImage);

          } else {
            image = mat8uc3_rgb;
          }
        } else {
          cv::Mat img(isizey, isizex, framebuf_encoding, framebuf->buf);
          if (applyCalib && subImage) {
            image = (img - *subImage);

          } else {
            image = img;
          }
        }
        if (performCalib) {
          if (image.data) {
            if (calibimages.size() < calibrationImages) {
              calibimages.push_back(new cv::Mat(image));
              RTCameraBaseLDBG_ << "Calibration images:" << calibimages.size()<<" "<<image.cols<<"x"<<image.rows;
              setState(NodeHealtDefinitionValue::NODE_HEALT_STATUS_CALIBRATE,false);
            } else {
              std::vector<unsigned char> encbuf;
              RTCameraBaseLDBG_ << "Average ";

              cv::Mat res=getMean(calibimages);
              RTCameraBaseLDBG_ << "Average size:"<<res.cols<<"x"<<res.rows;

              if(res.data){
                subImage = new cv::Mat(res);
                RTCameraBaseLDBG_ << "Calibrated image:" << subImage->cols<<"x"<<subImage->rows;

                for_each(calibimages.begin(),calibimages.end(),[](const cv::Mat* ele){delete ele;});
                calibimages.clear();
                
                bool code = cv::imencode(encoding, *subImage, encbuf, encode_params);
                if (code) {
                  uchar                            *ptr = reinterpret_cast<uchar *>(&(encbuf[0]));
                  chaos::common::data::CDataWrapper cal;
                  cal.addBinaryValue("FRAMEBUFFER", (const char *)ptr, encbuf.size());
                  saveData("calibration_image", cal);
                  applyCalib = true;
                  setStateVariableSeverity(StateVariableTypeAlarmCU, "calibration_error", chaos::common::alarm::MultiSeverityAlarmLevelClear);

                } else {
                  setStateVariableSeverity(StateVariableTypeAlarmCU, "calibration_error", chaos::common::alarm::MultiSeverityAlarmLevelHigh);

                }

                removeTag("CALIBRATION");
                RTCameraBaseLDBG_ << "END calibration";
                setState(NodeHealtDefinitionValue::NODE_HEALT_STATUS_START,true);
              }else {
                RTCameraBaseLERR_ << "Wrong resulting calibration image";

              }
              performCalib = false;

            }
          }
        }
      } catch (cv::Exception &ex) {
        std::stringstream ss;

        ss << "OpenCV Exception converting image:" << ex.what();
        if (applyCalib == false) {
          if (!stopCapture)
            goInFatalError(ss.str(), -1000, __PRETTY_FUNCTION__);
        }
        applyCalib = false;

      } catch (runtime_error &ex) {
        std::stringstream ss;

        ss << "Exception converting image:" << ex.what();
        if (!stopCapture)
          goInFatalError(ss.str(), -1000, __PRETTY_FUNCTION__);

        //  metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelError,ss.str());
      }
      //std::vector<unsigned char> *encbuf = NULL;
      //encoded_t                   ele;
      Encoder* ele=new Encoder();
      try {
        // bool code = cv::imencode(encoding, image, buf );
        // apply_zoom=(ZOOMX!=0.0) ||(ZOOMY!=0.0);
        if (apply_resize) {
          if ((isizex != imagesizex) || (isizey != imagesizey)) {
            cv::Mat res;
            cv::resize(image, res, Size(imagesizex, imagesizey), 0, 0);
            image = res;
          }
        }
        int rota = *rot % 360;
        if (rota) {
          image = rotate(image, rota);
        }
        filtering(image);
       // encbuf = new std::vector<unsigned char>();
        // RTCameraBaseLDBG_ << "Encoding "<<encoding<<" , row:"<<image.rows<<"
        // col:"<<image.cols<<" channels:"<<image.channels();
       /* 
       ele.sizex   = image.cols;
       ele.sizey   = image.rows;
       ele.offsetx = framebuf->offsetx;
       ele.offsety = framebuf->offsety;
       ele.ts= framebuf->ts;
        */
       ele->offsetx=framebuf->offsetx;
       ele->offsety=framebuf->offsety;
       ele->ts=framebuf->ts;

#ifdef CAMERA_GEN_VIDEO
        video.write(image);
#endif
       // bool code = cv::imencode(encoding, image, *encbuf, encode_params);
       bool code=ele->encode(encoding,image,encode_params);
        delete (framebuf);
        // image.deallocate();
        if (code == false) {
          setStateVariableSeverity(
              StateVariableTypeAlarmCU, "encode_error", chaos::common::alarm::MultiSeverityAlarmLevelHigh);
          RTCameraBaseLERR_ << "Encode error:" << fmt;
          delete ele;
          ele=NULL;
        //  encbuf = NULL;
        } else {
          setStateVariableSeverity(
              StateVariableTypeAlarmCU, "encode_error", chaos::common::alarm::MultiSeverityAlarmLevelClear);
         
          int encoderet;

          //ele.img = encbuf;
          //  RTCameraBaseLDBG_ << "pushing encode queue:"<<encodedImg[0].length();
        if(encodedImg[indx].length()==CAMERA_FRAME_BUFFERING){
           setStateVariableSeverity(
                   StateVariableTypeAlarmCU, "encodeQueue",
                   chaos::common::alarm::MultiSeverityAlarmLevelHigh);
            RTCameraBaseLERR_ << "Error encodeQueue queue "<<indx<<":" << encodedImg[indx].length();
            if(stopCapture){
                 continue;
          }
        } else if((encodedImg[indx].length()>0) && (encodedImg[indx].length()<(CAMERA_FRAME_BUFFERING-1))){
           setStateVariableSeverity(
                   StateVariableTypeAlarmCU, "encodeQueue",
                   chaos::common::alarm::MultiSeverityAlarmLevelWarning);
            RTCameraBaseLERR_ << "Warning encodeQueue queue "<<indx<<":"<< encodedImg[indx].length();

        } else {
          setStateVariableSeverity(
                   StateVariableTypeAlarmCU, "encodeQueue",
                   chaos::common::alarm::MultiSeverityAlarmLevelClear);
        }

          encoderet = encodedImg[indx].push(ele);
          

          if (encoderet < 0) {
            // FULL
             setStateVariableSeverity(
                   StateVariableTypeAlarmCU, "encodeQueue",
                   chaos::common::alarm::MultiSeverityAlarmLevelHigh);
            RTCameraBaseLDBG_ <<indx<<"] Encode Queue Error: " << encoderet << " queue:" << encodedImg[indx].length();
            continue;
          } 
           encode_time += (chaos::common::utility::TimingUtil::
                              getTimeStampInMicroseconds() -
                          start);
          counter_encode++;
        }
      } catch (cv::Exception &ex) {
        std::stringstream ss;
        if (!stopCapture) {
          ss << "OpenCV Exception Encoding format '"<<encoding<<"' :" << ex.what();
          goInFatalError(ss.str(), -1000, __PRETTY_FUNCTION__);
        }

        /*if (encbuf) {
          delete encbuf;
        }*/
        if(ele){
          delete ele;
        }

      } catch (std::runtime_error &e) {
        std::stringstream ss;
        ss << "Encode exception:" << e.what() << " orher:" << fmt;
     /*   if (encbuf) {
          delete encbuf;
        }*/
         if(ele){
          delete ele;
        }
        if (!stopCapture) {
          goInFatalError(ss.str(), -1001, __PRETTY_FUNCTION__);
        }

        usleep(100000);
      } catch (...) {
        std::stringstream ss;
        ss << "Uknown Encode exception:" << fmt;

       /* if (encbuf) {
          delete encbuf;
        }*/
         if(ele){
          delete ele;
        }
        if (!stopCapture) {
          usleep(100000);

          goInFatalError(ss.str(), -1002, __PRETTY_FUNCTION__);
        }
      }
      image.release();
    }
  }

  RTCameraBaseLDBG_ <<indx<<"] Encode thread exiting Queue: " << encodedImg[indx].length() << " has Stopped:" << hasStopped();

}
void calcXYFromAngle(double sx, double sy, double angle, double &x, double &y) {
  x = sqrt(pow(sx * cos(angle), 2) + pow(sy * sin(angle), 2));
  y = sqrt(pow(sx * sin(angle), 2) + pow(sy * cos(angle), 2));
}
void plotEllipse(Mat color, int32_t X_m, int32_t Y_m, int32_t S_x, int32_t S_y, int32_t r, const Scalar &col, int tick = 1) {
  double lx, ly;
  // calcXYFromAngle(S_x,S_y,rho,lx,ly);
  double a = S_x, b = S_y;
  double cx0  = X_m - a * cos(r);
  double cy0  = Y_m - a * sin(r);
  double cx1  = X_m + a * cos(r);
  double cy1  = Y_m + a * sin(r);
  double cmx0 = X_m + b * sin(r);
  double cmy0 = Y_m - b * cos(r);
  double cmx1 = X_m - b * sin(r);
  double cmy1 = Y_m + b * cos(r);
  line(color, Point(cx0, cy0), Point(cx1, cy1), col, 1);
  line(color, Point(cmx0, cmy0), Point(cmx1, cmy1), col, 1);

  ellipse(color, Point(X_m, Y_m), Size(S_x, S_y), r, 0, 360, col, tick, 8);
}
int RTCameraBase::filtering(cv::Mat &image) {
#ifdef CERN_ROOT

  if (performAutoReference) {
    setStateVariableSeverity(StateVariableTypeAlarmCU, "auto_reference_error", chaos::common::alarm::MultiSeverityAlarmLevelClear);
    performAutoReference = false;
    double a, x, y, sx, sy, rho;
    if (root::image::rootGaussianImage2dFit(image, fit_threshold, fit_level, a, x, y, sx, sy, rho) == 0) {
      applyReference = true;
      *refx          = (int)x;
      *refy          = (int)y;
      *refsx         = (int)sx;
      *refsy         = (int)sy;
      *refrho        = rho;
      *refabs        = false;

      getAttributeCache()->setInputDomainAsChanged();
      RTCameraBaseLDBG_ << "AUTO REFERENCE:(" << x << "," << y << ") sx:" << sx << " sy:" << sy << " rho:" << rho;
      pushInputDataset();
    } else {
      setStateVariableSeverity(StateVariableTypeAlarmCU, "auto_reference_error", chaos::common::alarm::MultiSeverityAlarmLevelWarning);
      RTCameraBaseLERR_ << "FAILED AUTO REFERENCE:(" << x << "," << y << ") sx:" << sx << " sy:" << sy << " rho:" << rho;
    }
  }
#endif

  if (applyReference && (*refsx) && (*refsy)) {
    int x = *refx, y = *refy;
    if (*refabs) {
      x = x - *offsetx;
      y = y - *offsety;
    }
    //  if((x>=0) && (y>=0)){
    // RTCameraBaseLDBG_<<"Ellipse:("<<x<<","<<y<<") sx:"<<*refsx<<" sy:"<<*refsy<<" rho:"<<*refrho<<"R:"<<refenceR<<",G:"<<refenceG<<",B:"<<refenceB<<" tick:"<<refenceThick;
    if ((refenceR != refenceG) || (refenceR != refenceB)) {
      if (image.channels() == 1) {
        #if (CV_VERSION_MAJOR >= 4)
         cvtColor(image, image,cv::COLOR_GRAY2BGR);
        #else
        cvtColor(image, image, cv::COLOR_GRAY2RGB);
        #endif
      }
    }
    plotEllipse(image, x, y, *refsx, *refsy, *refrho, Scalar(refenceR, refenceG, refenceB), refenceThick);
    //}
  }
  return 0;
}

chaos::common::data::CDWUniquePtr
RTCameraBase::unitPerformCalibration(chaos::common::data::CDWUniquePtr data) {
  applyCalib           = false;
  performAutoReference = false;
  fit_threshold        = 0;
  if (data.get()) {
      RTCameraBaseLDBG_ << "Peform calibration:" << data->getJSONString();

    if (data->hasKey("autoreference")) {
      performAutoReference = true;

      int threshold = 0;
      if (data->hasKey("threshold")) {
        fit_threshold = data->getInt32Value("threshold");
      }
      if (data->hasKey("fit_level")) {
        fit_level = data->getInt32Value("fit_level");
      }
      return chaos::common::data::CDWUniquePtr();
    }
    if (data->hasKey("calibration") && data->isBoolValue("calibration") && (data->getBoolValue("calibration") == false)) {
      applyCalib   = false;
      performCalib = false;
     
      return chaos::common::data::CDWUniquePtr();
    }
  }
   if (data->hasKey("samples")) {
        calibrationImages = data->getInt32Value("samples");
  }
  if (subImage) {
    delete subImage;
    subImage = NULL;
  }
   applyCalib   = false;

  calibimages.clear();
  std::stringstream ss;
  ss << "Perform calibration on " << calibrationImages << " images";

  metadataLogging(
      chaos::common::metadata_logging::StandardLoggingChannel::LogLevelInfo,
      ss.str());

  addTag("CALIBRATION");
  performCalib = true;
  return chaos::common::data::CDWUniquePtr();
}

//! Execute the Control Unit work
void RTCameraBase::unitRun() throw(chaos::CException) {
  //uchar *ptr;
  int encode_cnt=0;
  *pacquire = (mode != CAMERA_DISABLE_ACQUIRE);
  *ptrigger =
      (mode != CAMERA_DISABLE_ACQUIRE) && (mode != CAMERA_TRIGGER_CONTINOUS);
  // *ppulse = ((mode == CAMERA_TRIGGER_SINGLE) || (mode == CAMERA_TRIGGER_SOFT));
  *omode = mode;
  if (mode == CAMERA_DISABLE_ACQUIRE) {
    if (encodedImg[encode_cnt].length() == 0) {
      // getAttributeCache()->setOutputAttributeValue("FRAMEBUFFER", 0, 0);
      sleep(1);

      return;
    }
  }

  if (buffering > 0) {
    // get the output attribute pointer form the internal cache
    if ((encode_time > 0) && (capture_time > 0)) {
      *enc_frame_rate     = (1000000 * counter_encode / encode_time)*encode_thds;
      *capture_frame_rate = (1000000 * counter_capture / capture_time);
    }

    if ((counter_capture % 1000) == 0) {
      // RTCameraBaseLDBG_<<"CAPTURE TIME
      // (us):"<<capture_time/counter_capture<<"
      // HZ:"<<(1000000*counter_capture/capture_time)<<" ENCODE TIME
      // (us):"<<encode_time/counter_encode<<"
      // HZ:"<<(1000000*counter_encode/encode_time);
      capture_time    = 0;
      encode_time     = 0;
      counter_encode  = 0;
      counter_capture = 0;
    }
   // encoded_t                   ele;
   // std::vector<unsigned char> *a;
    Encoder*ele=NULL;
    // RTCameraBaseLDBG_ << "popping encode queue:"<<encodedImg[0].length();
    
    int ret;
    if(encode_thds>1){
            ret=encoded_queue.wait_and_pop(encode_cnt,0);
     }
    ret = encodedImg[encode_cnt].wait_and_pop(ele, 0);

    if(ret<0){
      RTCameraBaseLDBG_ << "popping encode queue "<<encode_cnt<<":"<<encodedImg[encode_cnt].length();

      return;
    }

    

    //if ((ret >= 0) && ele.img) {
    if (ele->data()) {

      //a = ele.img;
      //  RTCameraBaseLDBG_ << " Encode Queue:" << encodedImg[0].length()<< " Capture Queue:" << captureImg[0].length();

     // ptr       = ;//reinterpret_cast<uchar *>(&((*a)[0]));
      *osizex   = ele->sizex;
      *osizey   = ele->sizey;
      *ooffsetx = ele->offsetx;
      *ooffsety = ele->offsety;
      setOutputTimestamp(chaos::common::utility::TimingUtil::getTimeStamp());
#ifdef ZERO_COPY
      getAttributeCache()->setOutputAttributeValue("FRAMEBUFFER", ele,chaos::CHAOS_BUFFER_OWN_CALLEE);

#else   
      getAttributeCache()->setOutputAttributeValue("FRAMEBUFFER", ele->data(),ele->size());
#endif
      //RTCameraBaseLDBG_ << "push image:"<<a->size()<<" capture queue:"<<captureImg[0].length() <<" encode queue:"<<encodedImg[0].length()<<" lat:"<<(chaos::common::utility::TimingUtil::getTimeStamp()-ele.ts/1000)<<" ms";
      if(streamer&&cameraStreamEnable){
        streamer->publish(streamName,std::string( ( const char*)ele->data(), ele->size()));
      }
      setHigResolutionAcquistionTimestamp(ele->ts);
#ifdef CAMERA_ENABLE_SHARED

      if (shared_mem.get()) {
        shared_mem->writeMsg((void *)ptr, a->size());
        shared_mem->notify_all();
      }
#endif
#ifndef ZERO_COPY

  delete (ele);
#endif
      getAttributeCache()->setOutputDomainAsChanged();
    }
  } else {
    int           ret;
    camera_buf_t *img = 0;

    ret = driver->waitGrab(&img, trigger_timeout);
    if ((img != 0) && (ret > 0) && img->buf) {
      // keep alarm high
      setStateVariableSeverity(
          StateVariableTypeAlarmDEV, "capture_error", chaos::common::alarm::MultiSeverityAlarmLevelClear);
      setStateVariableSeverity(
          StateVariableTypeAlarmDEV, "capture_timeout", chaos::common::alarm::MultiSeverityAlarmLevelClear);
      setStateVariableSeverity(
          StateVariableTypeAlarmDEV, "camera_disconnect", chaos::common::alarm::MultiSeverityAlarmLevelClear);
      *osizex = img->width;
      *osizey = img->height;
      
      *ooffsetx = img->offsetx;
      *ooffsety = img->offsety;

      cv::Mat                     image(*osizey, *osizex, framebuf_encoding, (uint8_t *)img->buf);
     // std::vector<unsigned char> *encbuf = NULL;
      try {
        // bool code = cv::imencode(encoding, image, buf );
        // apply_zoom=(ZOOMX!=0.0) ||(ZOOMY!=0.0);
        if (apply_resize) {
          if ((*sizex != imagesizex) || (*sizey != imagesizey)) {
            cv::Mat res;
            cv::resize(image, res, Size(imagesizex, imagesizey), 0, 0);
            image = res;
          }
        }
        filtering(image);
     //   std::vector<unsigned char> encbuf;
          Encoder* enc=new Encoder();
          bool code= enc->encode(encoding,image,encode_params);
       // bool code = cv::imencode(encoding, image, encbuf);
        if (code == false) {
          setStateVariableSeverity(
              StateVariableTypeAlarmCU, "encode_error", chaos::common::alarm::MultiSeverityAlarmLevelHigh);
          RTCameraBaseLERR_ << "Encode error:" << fmt;
        } else {
          setStateVariableSeverity(
              StateVariableTypeAlarmCU, "encode_error", chaos::common::alarm::MultiSeverityAlarmLevelClear);
          //ptr = reinterpret_cast<uchar *>(&(encbuf[0]));
          //getAttributeCache()->setOutputAttributeValue("FRAMEBUFFER", ptr, encbuf.size());
          if(enc->data()){
#ifdef ZERO_COPY
            getAttributeCache()->setOutputAttributeValue("FRAMEBUFFER", enc,chaos::CHAOS_BUFFER_OWN_CALLEE);
#else
            getAttributeCache()->setOutputAttributeValue("FRAMEBUFFER", enc->data(),enc->size());

#endif
            if(streamer&&cameraStreamEnable){
                streamer->publish(streamName,std::string( ( const char*)enc->data(), enc->size()));
            }
          }
         // RTCameraBaseLDBG_ << "push image:"<<encbuf.size()<<" capture queue:"<<captureImg[0].length() <<" encode queue:"<<encodedImg[0].length()<<" lat:"<<(chaos::common::utility::TimingUtil::getTimeStampInMicroseconds()-img->ts)<<" us";

          setHigResolutionAcquistionTimestamp(img->ts);
#ifndef ZERO_COPY

      delete (enc);
#endif

#ifdef CAMERA_ENABLE_SHARED

          if (shared_mem.get()) {
            shared_mem->writeMsg((void *)enc->data(), encbuf->size());
            shared_mem->notify_all();
          }
#endif
          getAttributeCache()->setOutputDomainAsChanged();
        }
      } catch (...) {
        goInFatalError("Encode exception ", -1004, __PRETTY_FUNCTION__);
      }
    } else {
      switch(ret){
        case  chaos::ErrorCode::EC_GENERIC_TIMEOUT:
        RTCameraBaseLERR_ << " TIMEOUT:" << ret;
        setStateVariableSeverity(
            StateVariableTypeAlarmDEV, "capture_timeout", chaos::common::alarm::MultiSeverityAlarmLevelWarning);

        break;
        case CAMERA_DRIVER_DISCONNECT:
            RTCameraBaseLERR_ << " DISCONNECTED:" << ret;

        setStateVariableSeverity(
          StateVariableTypeAlarmDEV, "camera_disconnect", chaos::common::alarm::MultiSeverityAlarmLevelHigh);
        break;
        case CAMERA_BANDWIDTH_ERROR:
        RTCameraBaseLERR_ << " BANDWITH :" << ret;

        setStateVariableSeverity(
          StateVariableTypeAlarmDEV, "camera_bandwidth", chaos::common::alarm::MultiSeverityAlarmLevelHigh);
        break;
        default:
          RTCameraBaseLERR_ << " Error wait returned:" << ret;
        setStateVariableSeverity(
            StateVariableTypeAlarmDEV, "capture_error", chaos::common::alarm::MultiSeverityAlarmLevelHigh);
      }
          }
  }
}

//! Execute the Control Unit work
void RTCameraBase::unitStop() throw(chaos::CException) {
  
  RTCameraBaseLDBG_ << "Stop:" << hasStopped();
  stopGrabbing();
}

//! Deinit the Control Unit
void RTCameraBase::unitDeinit() throw(chaos::CException) {
  stopGrabbing();
  
  
  /*
  encodedImg[0].get().consume_all([this](encoded_t i) {
    delete (i.img);
  });
  */
 
  
  driver->cameraDeinit();
}

chaos::common::data::CDWUniquePtr RTCameraBase::getAction(chaos::common::data::CDWUniquePtr p) {
  chaos::common::data::CDataWrapper *camera_props = new chaos::common::data::CDataWrapper();
  int                                ret          = 0;
  if ((ret = driver->getCameraProperties(*camera_props)) != 0) {
    RTCameraBaseLERR_ << "Error retriving camera properties ret:" << ret;
  }
  return chaos::common::data::CDWUniquePtr(camera_props);
}
chaos::common::data::CDWUniquePtr RTCameraBase::setAction(chaos::common::data::CDWUniquePtr p) {
  return chaos::common::data::CDWUniquePtr();
}
void RTCameraBase::fatalErrorHandler(const chaos::CException &r) {
  stopGrabbing();
}
bool RTCameraBase::unitRestoreToSnapshot(chaos::cu::control_manager::AbstractSharedDomainCache *const snapshot_cache) throw(chaos::CException) {
  // check if in the restore cache we have all information we need
  /*  if (!snapshot_cache->getSharedDomain(DOMAIN_OUTPUT).hasAttribute("local")) {
        RESTORE_LERR << " missing 'local' to restore";
        return false;
    }*/

  chaos::common::data::CDataWrapper cd;
  snapshot_cache->getSharedDomain(DOMAIN_INPUT).exportToCDataWrapper(cd);

  INFO << "Restoring:" << cd.getJSONString();
  chaos::common::data::CDWUniquePtr p = cd.clone();
  driver->setDrvProperties(p);

  return true;
}

//! attribute changed handler
bool RTCameraBase::unitInputAttributePreChangeHandler(chaos::common::data::CDWUniquePtr &prop) {
  // in case of roi we must be assure that before the resize and then the offset.

  /*
  if(prop->hasKey("WIDTH")&&prop->hasKey("HEIGHT")&&prop->hasKey("OFFSETX")&&prop->hasKey("OFFSETY")){
    int ret = driver->setCameraProperty("WIDTH", prop->getInt32Value("WIDTH"));
      usleep(200000);
     ret |= driver->setCameraProperty("HEIGHT", prop->getInt32Value("HEIGHT"));
      usleep(200000);

     ret |= driver->setCameraProperty("OFFSETX", prop->getInt32Value("OFFSETX"));
     usleep(200000);

     ret |= driver->setCameraProperty("OFFSETY", prop->getInt32Value("OFFSETY"));
      usleep(200000);

    RTCameraBaseLDBG_ << "ROI " << ret;

      prop->removeKey("HEIGHT");
    prop->removeKey("WIDTH");
    prop->removeKey("OFFSETX");
    prop->removeKey("OFFSETY");


  }
  */

  return true;
}

/*
CDataWrapper *RTCameraBase::my_custom_action(CDataWrapper *actionParam, bool&
detachParam) { CDataWrapper *result =  new CDataWrapper(); return result;
}
*/
