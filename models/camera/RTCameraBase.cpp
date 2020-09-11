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
#include <driver/sensors/core/AbstractSensorDriver.h>
#include <driver/sensors/core/CameraDriverInterface.h>
#include <opencv2/core/core.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#define DECODE_CVENCODING(e, cvenc)                                            \
  if (e == #cvenc) {                                                           \
    RTCameraBaseLDBG_ << "Found Encoding:" << #cvenc;                          \
    framebuf_encoding = cvenc;                                                 \
    switch (cvenc) {                                                           \
    case CV_8UC4:                                                              \
    case CV_8SC4:                                                              \
      bpp = 4;                                                                 \
      break;                                                                   \
    case CV_8UC3:                                                              \
    case CV_8SC3:                                                              \
      bpp = 3;                                                                 \
      break;                                                                   \
    case CV_8UC2:                                                              \
    case CV_8SC2:                                                              \
      bpp = 2;                                                                 \
      break;                                                                   \
    case CV_8UC1:                                                              \
    case CV_8SC1:                                                              \
      bpp = 1;                                                                 \
      break;                                                                   \
    case CV_16UC1:                                                             \
    case CV_16SC1:                                                             \
      bpp = 2;                                                                 \
      break;                                                                   \
    case CV_16UC2:                                                             \
    case CV_16SC2:                                                             \
      bpp = 4;                                                                 \
      break;                                                                   \
    case CV_16UC3:                                                             \
    case CV_16SC3:                                                             \
      bpp = 6;                                                                 \
      break;                                                                   \
    case CV_32SC1:                                                             \
      bpp = 4;                                                                 \
      break;                                                                   \
    case CV_32SC2:                                                             \
      bpp = 8;                                                                 \
      break;                                                                   \
    case CV_32SC3:                                                             \
      bpp = 16;                                                                \
      break;                                                                   \
    default:                                                                   \
      bpp = 16;                                                                \
    }                                                                          \
  }
using namespace chaos;
using namespace chaos::common::data::cache;
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

RTCameraBase::RTCameraBase(const string &_control_unit_id,
                           const string &_control_unit_param,
                           const ControlUnitDriverList &_control_unit_drivers,
                           const int _buffering)
    : RTAbstractControlUnit(_control_unit_id, _control_unit_param,
                            _control_unit_drivers),
      driver(NULL), framebuf_encoding(CV_8UC3), buffering(_buffering),
      captureImg(_buffering), encodedImg(_buffering), encode_time(0),
      capture_time(0), network_time(0), captureQueue(0), encodeQueue(0),
      hw_trigger_timeout_us(5000000), sw_trigger_timeout_us(0), imagesizex(0),
      imagesizey(0), apply_resize(false), trigger_timeout(0), bpp(3),
      stopCapture(true), stopEncoding(true), isRunning(false), subImage(NULL),performCalib(false),
      applyCalib(false) {
  RTCameraBaseLDBG_ << "Creating " << _control_unit_id
                    << " params:" << _control_unit_param;

  framebuf = NULL;
  try {
    chaos::common::data::CDataWrapper p;
    p.setSerializedJsonData(_control_unit_param.c_str());
    if (p.hasKey(FRAMEBUFFER_ENCODING_KEY) &&
        p.isStringValue(FRAMEBUFFER_ENCODING_KEY)) {
      framebuf_encoding_s = p.getStringValue(FRAMEBUFFER_ENCODING_KEY);
      framebuf_encoding = fmt2cv(framebuf_encoding_s);
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
    if (p.hasKey("APPLY_CALIB")) {
      std::stringstream ss;
      std::string fname = getDeviceID();
      replace(fname.begin(), fname.end(), '/', '_');
      ss << fname << "_calib.png";

      cv::Mat sub = imread(ss.str());
      if (sub.data) {
        subImage = new cv::Mat(sub);
        RTCameraBaseLDBG_ << " Loading CALIB DATA:" << ss.str();
        applyCalib = true;
      } else {
        RTCameraBaseLERR_ << " CALIB DATA does not exists:" << ss.str();
      }
    }
    if (imagesizex > 0 && imagesizey > 0) {
      apply_resize = true;
      RTCameraBaseLDBG_ << "Camera will resize image to fit into: "
                        << imagesizex << "x" << imagesizey;
    }
  } catch (...) {
  }
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
void RTCameraBase::updateProperty() {
  std::vector<std::string> props;
  chaos::common::data::CDataWrapper camera_props;
  int ret = 0;
  if ((ret = driver->getCameraProperties(camera_props)) != 0) {
    RTCameraBaseLERR_ << "Error retriving camera properties ret:" << ret;
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

        if(*i == WIDTH_KEY){
          *sizex=tmp;
        }
        if(*i == HEIGHT_KEY){
          *sizey=tmp;
        }
        if(*i == OFFSETX_KEY){
          *offsetx=tmp;
        }if(*i == OFFSETY_KEY){
          *offsety=tmp;
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
#define SETINTPROP(name, n, value)                                             \
  if (name == #n) {                                                            \
    n = value;                                                                 \
    return true;                                                               \
  }
bool RTCameraBase::setProp(const std::string &name, const std::string &value,
                           uint32_t size) {
  return false;
}
bool RTCameraBase::setCamera(const std::string &name, bool value,
                             uint32_t size) {
  if (name == "ACQUIRE") {
    return setProp(TRIGGER_MODE_KEY, (value == false)
                                         ? CAMERA_DISABLE_ACQUIRE
                                         : CAMERA_TRIGGER_CONTINOUS);
  } else if (name == "TRIGGER") {
    return setProp(TRIGGER_MODE_KEY, (value == false) ? CAMERA_TRIGGER_CONTINOUS
                                                      : CAMERA_TRIGGER_HW_HI);

  } else if (name == "PULSE") {
    return setProp(TRIGGER_MODE_KEY, (value == false) ? CAMERA_TRIGGER_CONTINOUS
                                                      : CAMERA_TRIGGER_SINGLE);
  }
  return false;
}

bool RTCameraBase::setProp(const std::string &name, int32_t value,
                           uint32_t size) {
  int ret;
  int32_t valuer;
  AttributeSharedCacheWrapper *cc = getAttributeCache();

  RTCameraBaseLDBG_ << "SETTING IPROP:" << name << " VALUE:" << value;
  bool stopgrab =
      (((name == WIDTH_KEY) ) ||
       ((name == HEIGHT_KEY) ) ||
       ((name == OFFSETX_KEY) ) ||
       ((name == OFFSETY_KEY) ) ||
       ((name == TRIGGER_MODE_KEY) && (value == CAMERA_DISABLE_ACQUIRE)));
  
  //bool stopgrab = ((name == TRIGGER_MODE_KEY) && (value == CAMERA_DISABLE_ACQUIRE));

  if (stopgrab) {
    stopGrabbing();
  }
  ret = driver->setCameraProperty(name, value);
  setStateVariableSeverity(StateVariableTypeAlarmDEV, "error_setting_property",
                           chaos::common::alarm::MultiSeverityAlarmLevelClear);

  if (ret != 0) {
    std::stringstream ss;
    ss << "error setting \"" << name << "\" to " << value;
    setStateVariableSeverity(
        StateVariableTypeAlarmDEV, "error_setting_property",
        chaos::common::alarm::MultiSeverityAlarmLevelWarning);
    metadataLogging(
        chaos::common::metadata_logging::StandardLoggingChannel::LogLevelError,
        ss.str());
  }

  if ((name == TRIGGER_MODE_KEY)) {
    // updateProperty();
    driver->getCameraProperty(name, valuer);
    int32_t *pmode = cc->getRWPtr<int32_t>(DOMAIN_OUTPUT, name);

    mode = *pmode = valuer;
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
  RTCameraBaseLDBG_ << "SET IPROP:" << name << " SET VALUE:" << value
                    << " READ VALUE:" << valuer << " ret:" << ret;
  if (stopgrab &&
      !((name == TRIGGER_MODE_KEY) && (value == CAMERA_DISABLE_ACQUIRE))) {
    startGrabbing();
  } else if (stopCapture && ((name == TRIGGER_MODE_KEY) &&
                             (value != CAMERA_DISABLE_ACQUIRE))) {
    startGrabbing();
  }
  getAttributeCache()->setInputDomainAsChanged();
  getAttributeCache()->setOutputDomainAsChanged();
  return (ret == 0);
}

bool RTCameraBase::setProp(const std::string &name, double value,
                           uint32_t size) {
  int ret;
  double valuer;
  RTCameraBaseLDBG_ << "SET FPROP:" << name << " VALUE:" << value << endl;
  // SETINTPROP(name,ZOOMX,value);
  // SETINTPROP(name,ZOOMY,value);

  ret = driver->setCameraProperty(name, value);
  setStateVariableSeverity(StateVariableTypeAlarmDEV, "error_setting_property",
                           chaos::common::alarm::MultiSeverityAlarmLevelClear);

  if (ret != 0) {
    setStateVariableSeverity(
        StateVariableTypeAlarmDEV, "error_setting_property",
        chaos::common::alarm::MultiSeverityAlarmLevelWarning);
  }
  // updateProperty();
  driver->getCameraProperty(name, valuer);
  AttributeSharedCacheWrapper *cc = getAttributeCache();
  double *p = cc->getRWPtr<double>(DOMAIN_OUTPUT, name);
  *p = valuer;
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
          -1, "No associated camera driver for:" + getDeviceID(),
          __PRETTY_FUNCTION__);
    }
    driver = new CameraDriverInterface(acc);
    if (driver == NULL) {
      throw chaos::CException(-1, "cannot create driver for:" + getDeviceID(),
                              __PRETTY_FUNCTION__);
    }
  }
  if (driver->getCameraProperties(camera_props) != 0) {
    throw chaos::CException(-1, "Error retrieving camera properties",
                            __PRETTY_FUNCTION__);
  }
  std::vector<std::string> props;
  camera_props.getAllKey(props);

  /****
   *
   */
  addAttributeToDataSet("FMT", "image format (jpg,png,gif...)",
                        chaos::DataType::TYPE_STRING,
                        chaos::DataType::Bidirectional, 16);
  if (buffering > 1) {
    addAttributeToDataSet("CAPTURE_FRAMERATE", "Capture Frame Rate",
                          chaos::DataType::TYPE_INT32, chaos::DataType::Output);
    addAttributeToDataSet("ENCODE_FRAMERATE", "Encode Frame Rate",
                          chaos::DataType::TYPE_INT32, chaos::DataType::Output);
  }
  addAttributeToDataSet("ACQUIRE", "Is Acquiring",
                        chaos::DataType::TYPE_BOOLEAN,
                        chaos::DataType::Bidirectional);
  addAttributeToDataSet("TRIGGER", "Is triggered ",
                        chaos::DataType::TYPE_BOOLEAN,
                        chaos::DataType::Bidirectional);
  addAttributeToDataSet("PULSE", "Is pulse", chaos::DataType::TYPE_BOOLEAN,
                        chaos::DataType::Bidirectional);
  addHandlerOnInputAttributeName<::driver::sensor::camera::RTCameraBase, bool>(
      this, &::driver::sensor::camera::RTCameraBase::setCamera, "ACQUIRE");
  addHandlerOnInputAttributeName<::driver::sensor::camera::RTCameraBase, bool>(
      this, &::driver::sensor::camera::RTCameraBase::setCamera, "TRIGGER");
  addHandlerOnInputAttributeName<::driver::sensor::camera::RTCameraBase, bool>(
      this, &::driver::sensor::camera::RTCameraBase::setCamera, "PULSE");

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
        addHandlerOnInputAttributeName<::driver::sensor::camera::RTCameraBase,
                                       double>(
            this, &::driver::sensor::camera::RTCameraBase::setProp, *i);
      }
      if (camera_props.getValueType(*i) == chaos::DataType::TYPE_INT32) {
        addHandlerOnInputAttributeName<::driver::sensor::camera::RTCameraBase,
                                       int32_t>(
            this, &::driver::sensor::camera::RTCameraBase::setProp, *i);
      }
    }
  }

  addBinaryAttributeAsMIMETypeToDataSet("FRAMEBUFFER", "output image",
                                        "image/png", chaos::DataType::Output);

  /*addBinaryAttributeAsSubtypeToDataSet(
      "FRAMEBUFFER", "image", chaos::DataType::SUB_TYPE_CHAR,
      DEFAULT_RESOLUTION, chaos::DataType::Output);*/
  addStateVariable(
      StateVariableTypeAlarmDEV, "error_setting_property",
      "the operation in not supported i.e property not present or accesible");
  addStateVariable(StateVariableTypeAlarmCU, "encode_error",
                   "an error occurred during encode", LOG_FREQUENCY);
  addStateVariable(StateVariableTypeAlarmDEV, "capture_error",
                   "an error occurred during capture", LOG_FREQUENCY);
  addStateVariable(StateVariableTypeAlarmDEV, "capture_timeout",
                   "a timeout has occurred", LOG_FREQUENCY);

  addStateVariable(StateVariableTypeAlarmDEV, "captureQueueFull",
                   "Queue Capture Full", LOG_FREQUENCY);
  addStateVariable(StateVariableTypeAlarmDEV, "encodeQueueFull",
                   "Queue Encode Full", LOG_FREQUENCY);
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
  // getAttributeCache()->setCustomDomainAsChanged();
  // pushCustomDataset();
}

//! Initialize the Custom Control Unit
void RTCameraBase::unitInit() throw(chaos::CException) {
  int ret;
  int32_t itype;
  int32_t width, height;

  AttributeSharedCacheWrapper *cc = getAttributeCache();
  performCalib=false;
  // this properties must exist
  //
  sizex = cc->getRWPtr<int32_t>(DOMAIN_INPUT, WIDTH_KEY);
  sizey = cc->getRWPtr<int32_t>(DOMAIN_INPUT, HEIGHT_KEY);
  offsetx = cc->getRWPtr<int32_t>(DOMAIN_INPUT, OFFSETX_KEY);
  offsety = cc->getRWPtr<int32_t>(DOMAIN_INPUT, OFFSETY_KEY);
  //
  mode = *cc->getROPtr<int32_t>(DOMAIN_INPUT, TRIGGER_MODE_KEY);
  omode = cc->getRWPtr<int32_t>(DOMAIN_OUTPUT, TRIGGER_MODE_KEY);
  //   camera_out=cc->getRWPtr<uint8_t>(DOMAIN_OUTPUT, "FRAMEBUFFER");
  fmt = cc->getRWPtr<char>(DOMAIN_INPUT, "FMT");
  ofmt = cc->getRWPtr<char>(DOMAIN_OUTPUT, "FMT");
  pacquire = cc->getRWPtr<bool>(DOMAIN_OUTPUT, "ACQUIRE");
  ptrigger = cc->getRWPtr<bool>(DOMAIN_OUTPUT, "TRIGGER");
  ppulse = cc->getRWPtr<bool>(DOMAIN_OUTPUT, "PULSE");
  *pacquire = false;
  *ptrigger = false;
  *ppulse = false;

  if (buffering > 1) {
    capture_frame_rate =
        cc->getRWPtr<int32_t>(DOMAIN_OUTPUT, "CAPTURE_FRAMERATE");
    enc_frame_rate = cc->getRWPtr<int32_t>(DOMAIN_OUTPUT, "ENCODE_FRAMERATE");
  }

  // breanch number and soft reset

  if ((ret = driver->cameraInit(0, 0)) != 0) {
    throw chaos::CException(ret, "cannot initialize camera",
                            __PRETTY_FUNCTION__);
  }
  // not needed any mode
  /*
  std::vector<std::string> props;
    camera_props.getAllKey(props);
    for (std::vector<std::string>::iterator i = props.begin(); i != props.end();
         i++) {
      if (!camera_props.isCDataWrapperValue(*i)) {

        if (camera_props.getValueType(*i) == chaos::DataType::TYPE_DOUBLE) {

          double tmp = cc->getValue<double>(DOMAIN_INPUT, *i);
          if (tmp != 0) {
            /// to check!!!
            RTCameraBaseLDBG_ << "Init Double \"" << *i
                              << "\" from input:" << tmp;

            setProp(*i, tmp, 0);
          }
        }
        if (camera_props.getValueType(*i) == chaos::DataType::TYPE_INT32) {
          int32_t tmp = cc->getValue<int32_t>(DOMAIN_INPUT, *i);
          if (tmp != 0) {
            RTCameraBaseLDBG_ << "Init Integer \"" << *i
                              << "\" from input:" << tmp;

            setProp(*i, tmp, 0);
          }
        }
      }
    }

    for (std::vector<std::string>::iterator i = props.begin(); i != props.end();
         i++) {
      if (!camera_props.isCDataWrapperValue(*i)) {

        if (camera_props.getValueType(*i) == chaos::DataType::TYPE_DOUBLE) {
          double tmp, *p;

          driver->getCameraProperty(*i, tmp);
          p = cc->getRWPtr<double>(DOMAIN_INPUT, *i);
          *p = tmp;
        }
        if (camera_props.getValueType(*i) == chaos::DataType::TYPE_INT32) {
          int32_t tmp, *p;
          driver->getCameraProperty(*i, tmp);
          p = cc->getRWPtr<int32_t>(DOMAIN_INPUT, *i);
          *p = tmp;
        }
      }
    }
  */
  if (driver->getImageProperties(width, height, itype) == 0) {
    RTCameraBaseLDBG_ << "CAMERA IMAGE:" << width << "x" << height;
    *sizex = width;
    *sizey = height;
  }
  chaos::common::data::CDataWrapper cam_prop;
  if (driver->getCameraProperties(cam_prop) == 0) {
    if (cam_prop.hasKey(FRAMEBUFFER_ENCODING_KEY) &&
        cam_prop.isStringValue(FRAMEBUFFER_ENCODING_KEY)) {
      framebuf_encoding_s = cam_prop.getStringValue(FRAMEBUFFER_ENCODING_KEY);
      framebuf_encoding = fmt2cv(framebuf_encoding_s);
      std::string enc;
      bpp = cv2fmt(framebuf_encoding, enc);
    }
    if (cam_prop.hasKey(OFFSETX_KEY) && cam_prop.isInt32Value(OFFSETX_KEY)) {
      *offsetx = cam_prop.getInt32Value(OFFSETX_KEY);
    }
    if (cam_prop.hasKey(OFFSETY_KEY) && cam_prop.isInt32Value(OFFSETY_KEY)) {
      *offsety = cam_prop.getInt32Value(OFFSETY_KEY);
    }
  }
  if ((*sizex == 0) || (*sizey == 0)) {
    std::stringstream ss;
    ss << "Invalid image sizes:" << *sizex << "x" << *sizey;

    throw chaos::CException(-3, ss.str(), __PRETTY_FUNCTION__);
  }
  if (!apply_resize) {
    imagesizex = *sizex;
    imagesizey = *sizey;
  }
  cc->setOutputAttributeNewSize("FRAMEBUFFER", imagesizex * imagesizey * (bpp),
                                true);

  // framebuf = (uint8_t*)malloc(size);
  bpp = cv2fmt(framebuf_encoding, framebuf_encoding_s);

  RTCameraBaseLDBG_ << "Starting acquiring imagex " << *sizex << "x" << *sizey
                    << " bpp:" << bpp
                    << " framebuf_encoding:" << framebuf_encoding_s << " ("
                    << framebuf_encoding << ")";
  if (*fmt == 0) {
    strcpy(encoding, ".png");

  } else {
    snprintf(encoding, sizeof(encoding), ".%s", fmt);
  }
  strncpy(ofmt, encoding, sizeof(encoding));
  updateProperty();

  getAttributeCache()->setInputDomainAsChanged();
  getAttributeCache()->setOutputDomainAsChanged();
  getAttributeCache()->setCustomDomainAsChanged();

   pushCustomDataset();
}
void RTCameraBase::cameraGrabCallBack(const void *buf, uint32_t blen,
                                      uint32_t width, uint32_t heigth,
                                      uint32_t error) {}
void RTCameraBase::startGrabbing() {
  if (isRunning == false) {
    return;
  }

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
  RTCameraBaseLDBG_ << "Allocated " << buffering << " buffers of "
                    << (*sizex * *sizey * bpp) << " bytes";

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

  driver->startGrab(0);
  if (stopCapture == true) {
    stopCapture = false;
    if (buffering > 1) {
      capture_th = boost::thread(&RTCameraBase::captureThread, this);
    }
  }
  getAttributeCache()->setCustomDomainAsChanged();
  pushCustomDataset();
}
void RTCameraBase::haltThreads() {
  RTCameraBaseLDBG_ << "Stop Grab low level" << stopCapture;
  stopCapture = true;
  driver->stopGrab();
}

void RTCameraBase::stopGrabbing() {
  RTCameraBaseLDBG_ << "Stop Grabbing..." << stopCapture;
  haltThreads();
  if (buffering > 1) {

   /* metadataLogging(
        chaos::common::metadata_logging::StandardLoggingChannel::LogLevelInfo,
        "Stop grabbing");
*/
    wait_capture.notify_all();
    wait_encode.notify_all();
    full_encode.notify_all();
    full_capture.notify_all();
    if (boost::this_thread::get_id() != encode_th.get_id()) {
      encode_th.join();
    }
    if (boost::this_thread::get_id() != capture_th.get_id()) {
      capture_th.join();
    }
  }
  RTCameraBaseLDBG_ << "Stop Grabbing done:" << stopCapture;

  /*    for(int cnt=0;cnt<CAMERA_FRAME_BUFFERING;cnt++){
        free(framebuf_out[cnt].buf);
        framebuf_out[cnt].size=0;
        framebuf_out[cnt].buf=NULL;
    }*/

  setStateVariableSeverity(StateVariableTypeAlarmDEV, "error_setting_property",
                           chaos::common::alarm::MultiSeverityAlarmLevelClear);
  setStateVariableSeverity(StateVariableTypeAlarmDEV, "capture_error",
                           chaos::common::alarm::MultiSeverityAlarmLevelClear);
  setStateVariableSeverity(StateVariableTypeAlarmDEV, "capture_timeout",
                           chaos::common::alarm::MultiSeverityAlarmLevelClear);
  setStateVariableSeverity(StateVariableTypeAlarmCU, "encode_error",
                           chaos::common::alarm::MultiSeverityAlarmLevelClear);
  setStateVariableSeverity(StateVariableTypeAlarmCU, "encodeQueueFull",
                           chaos::common::alarm::MultiSeverityAlarmLevelClear);
  setStateVariableSeverity(StateVariableTypeAlarmCU, "captureQueueFull",
                           chaos::common::alarm::MultiSeverityAlarmLevelClear);
}
//! Execute the work, this is called with a determinated delay, it must be as
//! fast as possible
void RTCameraBase::unitStart() throw(chaos::CException) {
  isRunning = true;
  startGrabbing();
}

void RTCameraBase::captureThread() {
  int ret;
  const char *img = 0;
  uint64_t start;
  counter_capture = 0;
  RTCameraBaseLDBG_ << "Capture thread STARTED";
  encode_th = boost::thread(&RTCameraBase::encodeThread, this);

  while (!stopCapture) {
    img = 0;
    start = chaos::common::utility::TimingUtil::getTimeStampInMicroseconds();
    ret = driver->waitGrab(&img, trigger_timeout);
    // RTCameraBaseLDBG_ << "grabbing ret:"<<ret;

    if ((img != 0) && (ret > 0)) {
      // keep alarm high
      setStateVariableSeverity(
          StateVariableTypeAlarmDEV, "capture_error",
          chaos::common::alarm::MultiSeverityAlarmLevelClear);
      setStateVariableSeverity(
          StateVariableTypeAlarmDEV, "capture_timeout",
          chaos::common::alarm::MultiSeverityAlarmLevelClear);

      buf_t data;
      data.buf = (uint8_t *)malloc(ret);
      if (data.buf == NULL) {
        throw chaos::CException(-1, "Heap memory exahusted",
                                __PRETTY_FUNCTION__);
      }
      data.size = ret;
      memcpy((void *)data.buf, img, ret);
      capture_time +=
          (chaos::common::utility::TimingUtil::getTimeStampInMicroseconds() -
           start);
      counter_capture++;
      bool pushret;
      int retry = 3;
      do {
        pushret = captureImg.push(data);
        if (pushret == false) {

          /*  setStateVariableSeverity(
                StateVariableTypeAlarmDEV, "captureQueueFull",
                chaos::common::alarm::MultiSeverityAlarmLevelWarning);
    */
          wait_capture.notify_one();
          RTCameraBaseLDBG_ << "Capture FULL Queue:" << captureQueue
                            << " Waiting...";

          boost::mutex::scoped_lock lock(mutex_io);
          // boost::chrono::microseconds period(5000000);
          boost::system_time const timeout =
              boost::get_system_time() + boost::posix_time::milliseconds(5000);

          //   boost::chrono::system_clock::time_point wakeUpTime =
          //   boost::chrono::system_clock::now() + period;

          full_capture.timed_wait(lock, timeout);
        } else {
          if (captureQueue < (buffering - 1)) {
            /* setStateVariableSeverity(
               StateVariableTypeAlarmDEV, "captureQueueFull",
               chaos::common::alarm::MultiSeverityAlarmLevelClear);*/
          }
          RTCameraBaseLDBG_ << "Capture Queue:" << captureQueue;
          captureQueue++;
          wait_capture.notify_one();
        }
      } while ((pushret == false) && (!stopCapture));

    } else {
      if (stopCapture == false) {
        if (ret == TRIGGER_TIMEOUT_ERROR) {
          RTCameraBaseLERR_ << " wait returned:" << ret
                            << " timeout stopped:" << stopCapture;
          setStateVariableSeverity(
              StateVariableTypeAlarmDEV, "capture_timeout",
              chaos::common::alarm::MultiSeverityAlarmLevelWarning);

        } else if (ret==CAMERA_GRAB_STOP){
            metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelError,"unexplectly driver stopped grabbing");
            haltThreads();
            setStateVariableSeverity(
              StateVariableTypeAlarmDEV, "capture_error",
              chaos::common::alarm::MultiSeverityAlarmLevelHigh);
        } else if (ret < 0) {
          RTCameraBaseLERR_ << " Error wait returned:" << ret;
          setStateVariableSeverity(
              StateVariableTypeAlarmDEV, "capture_error",
              chaos::common::alarm::MultiSeverityAlarmLevelHigh);
          usleep(100000);
        }
      }
    }
  }

  RTCameraBaseLDBG_ << "Capture thread ENDED Queue:" << captureQueue;
}
void RTCameraBase::encodeThread() {
  uint64_t start;
  counter_encode = 0;
  // bpp = cv2fmt(framebuf_encoding, framebuf_encoding_s);

  RTCameraBaseLDBG_ << "Encode " << encoding
                    << " , assuming framebuf:" << framebuf_encoding_s << "("
                    << framebuf_encoding << ") bpp:" << bpp
                    << " thread STARTED";
  while (!stopCapture) {

    buf_t a;

    if (captureImg.pop(a) && a.buf) {
      start = chaos::common::utility::TimingUtil::getTimeStampInMicroseconds();
      captureQueue--;

      full_capture.notify_one();
      if (*sizey * *sizex * bpp > a.size) {
        std::stringstream ss;

        ss << "Cannot encode an image bigger " << *sizex << "x" << *sizey
           << " bpp:" << bpp << " =" << (*sizex * *sizey * bpp)
           << " than size allocated:" << a.size
           << " framebuf encoding:" << framebuf_encoding_s;
        free(a.buf);
        /*    setStateVariableSeverity(
                  StateVariableTypeAlarmCU, "encode_error",
                  chaos::common::alarm::MultiSeverityAlarmLevelHigh);
            metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelError,ss.str());
            */
        goInFatalError(ss.str(), -1000, __PRETTY_FUNCTION__);
        continue;
        //  throw chaos::CException(-3, ss.str(), __PRETTY_FUNCTION__);
      }
      cv::Mat image;
      try {

      if (framebuf_encoding == cv::COLOR_BayerBG2RGB) {

        //   cv::Mat mat16uc1(*sizey, *sizex, CV_16UC1, a.buf);
        cv::Mat mat8uc1(*sizey, *sizex, CV_8UC1, a.buf);
        // cv::Mat mat16uc3_rgb(*sizey, *sizex, CV_16UC3);
        // cv::cvtColor(mat16uc1, mat16uc3_rgb, cv::COLOR_BayerBG2RGB);
        cv::Mat mat8uc3_rgb(*sizey, *sizex, CV_8UC3);
        cv::cvtColor(mat8uc1, mat8uc3_rgb, cv::COLOR_BayerBG2RGB);

        // mat16uc3_rgb.convertTo(mat8uc3_rgb, CV_8UC3);
        if (applyCalib && subImage) {
          image = (mat8uc3_rgb - *subImage);

        } else {
          image = mat8uc3_rgb;
        }

      } else if (framebuf_encoding == cv::COLOR_YUV2RGB_NV21) {
        cv::Mat mat16uc1 = cv::Mat(*sizey, *sizex, CV_8UC2, a.buf);
        //cv::Mat mat16uc3_rgb(*sizey, *sizex, CV_16UC3);

        cv::Mat mat8uc3_rgb(*sizey, *sizex, CV_8UC3);
       // cv::cvtColor(mat16uc1, mat8uc3_rgb, cv::COLOR_YUV2RGB );//
        cv::cvtColor(mat16uc1, mat8uc3_rgb, cv::COLOR_YUV2RGB_UYVY);//

        if (applyCalib && subImage) {
          image = (mat8uc3_rgb - *subImage);

        } else {
          image = mat8uc3_rgb;
        }
      } else {
        cv::Mat img(*sizey, *sizex, framebuf_encoding, a.buf);
        if (applyCalib && subImage) {
          image = (img - *subImage);

        } else {
          image = img;
        }
      }
      if (performCalib) {
        RTCameraBaseLDBG_ << "Calibrate";
        if (image.data) {
          if (subImage) {
            delete subImage;
          }
          subImage = new cv::Mat(image);
          std::stringstream ss;
          std::string fname = getDeviceID();
          replace(fname.begin(), fname.end(), '/', '_');
          ss << fname << "_calib.png";
            imwrite(ss.str(), image);
            metadataLogging(chaos::common::metadata_logging::
                                StandardLoggingChannel::LogLevelInfo,
                            ss.str());
          
        }
        performCalib = false;
      }
      } catch (cv::Exception& ex){
          std::stringstream ss;

            ss << "OpenCV Exception converting image:" << ex.what();
            goInFatalError(ss.str(), -1000, __PRETTY_FUNCTION__);

        
      } catch (runtime_error &ex) {
                std::stringstream ss;

            ss << "Exception converting image:" << ex.what();
            goInFatalError(ss.str(), -1000, __PRETTY_FUNCTION__);

            //  metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelError,ss.str());
      }
      std::vector<unsigned char> *encbuf = NULL;
      encoded_t ele;

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
        encbuf = new std::vector<unsigned char>();
        // RTCameraBaseLDBG_ << "Encoding "<<encoding<<" , row:"<<image.rows<<"
        // col:"<<image.cols<<" channels:"<<image.channels();

        bool code = cv::imencode(encoding, image, *encbuf);
        free(a.buf);
        // image.deallocate();
        if (code == false) {
          setStateVariableSeverity(
              StateVariableTypeAlarmCU, "encode_error",
              chaos::common::alarm::MultiSeverityAlarmLevelHigh);
          RTCameraBaseLERR_ << "Encode error:" << fmt;
          delete encbuf;

          encbuf = NULL;
        } else {
          setStateVariableSeverity(
              StateVariableTypeAlarmCU, "encode_error",
              chaos::common::alarm::MultiSeverityAlarmLevelClear);
          wait_encode.notify_one();
          encode_time += (chaos::common::utility::TimingUtil::
                              getTimeStampInMicroseconds() -
                          start);
          counter_encode++;
          bool encoderet;
          do {
            ele.img = encbuf;
            encoderet = encodedImg.push(ele);
            if (encoderet == false) {
              // FULL
              /* setStateVariableSeverity(
                   StateVariableTypeAlarmDEV, "encodeQueueFull",
                   chaos::common::alarm::MultiSeverityAlarmLevelWarning);*/
              RTCameraBaseLDBG_ << "Encode Queue FULL: " << encodeQueue
                                << " Waiting";

              boost::mutex::scoped_lock lock(mutex_encode);
              boost::system_time const timeout =
                  boost::get_system_time() +
                  boost::posix_time::milliseconds(5000);

              full_encode.timed_wait(lock, timeout);

            } else {
              if (encodeQueue < (buffering - 1)) {
                // keep encode high
                /* setStateVariableSeverity(
                   StateVariableTypeAlarmDEV, "encodeQueueFull",
                   chaos::common::alarm::MultiSeverityAlarmLevelClear);*/
              }
              encodeQueue++;
            }
          } while ((encoderet == false) && (!stopCapture));
        }
      } catch (cv::Exception& ex){
          std::stringstream ss;

            ss << "OpenCV Exception Encoding format:" << ex.what();
            goInFatalError(ss.str(), -1000, __PRETTY_FUNCTION__);

        if (encbuf) {
          delete encbuf;
        }

      } catch (std::runtime_error &e) {
        std::stringstream ss;
        ss << "Encode exception:" << e.what() << " orher:" << fmt;
        if (encbuf) {
          delete encbuf;
        }

        goInFatalError(ss.str(), -1001, __PRETTY_FUNCTION__);

        usleep(100000);
      } catch (...) {
        std::stringstream ss;
        ss << "Uknown Encode exception:" << fmt;

        if (encbuf) {
          delete encbuf;
        }
        usleep(100000);
        goInFatalError(ss.str(), -1002, __PRETTY_FUNCTION__);
      }
      image.release();

    } else {
      RTCameraBaseLDBG_ << "Encode EMPTY";

      boost::mutex::scoped_lock lock(mutex_encode);
      boost::system_time const timeout =
          boost::get_system_time() + boost::posix_time::milliseconds(5000);

      wait_capture.timed_wait(lock, timeout);
    }
  }
  boost::mutex::scoped_lock lock(mutex_encode);

  RTCameraBaseLDBG_ << "Encode thread exiting Queue: " << encodeQueue;

  /*
  encodedImg.consume_all([this](encoded_t i) {
     delete (i.img);
     encodeQueue--;
   });

   RTCameraBaseLDBG_ << "Encode thread ENDED Queue: " << encodeQueue;
   */
}

int RTCameraBase::filtering(cv::Mat &image) { return 0; }

chaos::common::data::CDWUniquePtr
RTCameraBase::unitPerformCalibration(chaos::common::data::CDWUniquePtr data) {
  applyCalib = false;
  performCalib = true;
  RTCameraBaseLDBG_ << "Perform calibration";

  while (performCalib) {
    usleep(100000);
  }
  if (!(data.get() && data->hasKey("disable"))) {
    applyCalib = true;
  }
  RTCameraBaseLDBG_ << "END calibration";
  return chaos::common::data::CDWUniquePtr();
}

//! Execute the Control Unit work
void RTCameraBase::unitRun() throw(chaos::CException) {
  uchar *ptr;
  const char *img = 0;
  *pacquire = (mode != CAMERA_DISABLE_ACQUIRE);
  *ptrigger =
      (mode != CAMERA_DISABLE_ACQUIRE) && (mode != CAMERA_TRIGGER_CONTINOUS);
  *ppulse = ((mode == CAMERA_TRIGGER_SINGLE) || (mode == CAMERA_TRIGGER_SOFT));
  *omode = mode;
  if (mode == CAMERA_DISABLE_ACQUIRE) {

    getAttributeCache()->setOutputAttributeValue("FRAMEBUFFER", 0, 0);
    getAttributeCache()->setOutputDomainAsChanged();
    sleep(1);
  }
  if (buffering > 1) {
    // get the output attribute pointer form the internal cache
    if ((encode_time > 0) && (capture_time > 0)) {
      *enc_frame_rate = (1000000 * counter_encode / encode_time);
      *capture_frame_rate = (1000000 * counter_capture / capture_time);
    }

    if ((counter_capture % 100000) == 0) {
      // RTCameraBaseLDBG_<<"CAPTURE TIME
      // (us):"<<capture_time/counter_capture<<"
      // HZ:"<<(1000000*counter_capture/capture_time)<<" ENCODE TIME
      // (us):"<<encode_time/counter_encode<<"
      // HZ:"<<(1000000*counter_encode/encode_time);
      capture_time = 0;
      encode_time = 0;
      counter_encode = 0;
      counter_capture = 0;
    }
    encoded_t ele;
    std::vector<unsigned char> *a;
    if (encodedImg.pop(ele)) {
      a = ele.img;
      encodeQueue--;
      /*  RTCameraBaseLDBG_ << " Encode Queue:" << encodeQueue
                          << " Capture Queue:" << captureQueue;
        */
      ptr = reinterpret_cast<uchar *>(&((*a)[0]));
      getAttributeCache()->setOutputAttributeValue("FRAMEBUFFER", ptr,
                                                   a->size());
      if (shared_mem.get()) {
        shared_mem->writeMsg((void *)ptr, a->size());
        shared_mem->notify_all();
      }
      delete (a);

      getAttributeCache()->setOutputDomainAsChanged();
      full_encode.notify_one();

    } else {
      // RTCameraBaseLDBG_<<"Encode EMPTY :"<<encodeWritePointer<<"
      // CaptureWrite:"<<captureWritePointer;
      full_encode.notify_one();

      boost::mutex::scoped_lock lock(mutex_encode);
      boost::system_time const timeout =
          boost::get_system_time() + boost::posix_time::milliseconds(5000);

      wait_encode.timed_wait(lock, timeout);
    }
  } else {
    int ret;
    ret = driver->waitGrab(&img, trigger_timeout);
    if ((img != 0) && (ret > 0)) {
      // keep alarm high
      setStateVariableSeverity(
          StateVariableTypeAlarmDEV, "capture_error",
          chaos::common::alarm::MultiSeverityAlarmLevelClear);
      setStateVariableSeverity(
          StateVariableTypeAlarmDEV, "capture_timeout",
          chaos::common::alarm::MultiSeverityAlarmLevelClear);
      cv::Mat image(*sizey, *sizex, framebuf_encoding, (uint8_t *)img);
      std::vector<unsigned char> *encbuf = NULL;
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
        std::vector<unsigned char> encbuf;

        bool code = cv::imencode(encoding, image, encbuf);
        if (code == false) {
          setStateVariableSeverity(
              StateVariableTypeAlarmCU, "encode_error",
              chaos::common::alarm::MultiSeverityAlarmLevelHigh);
          RTCameraBaseLERR_ << "Encode error:" << fmt;
        } else {
          setStateVariableSeverity(
              StateVariableTypeAlarmCU, "encode_error",
              chaos::common::alarm::MultiSeverityAlarmLevelClear);
          ptr = reinterpret_cast<uchar *>(&(encbuf[0]));
          getAttributeCache()->setOutputAttributeValue("FRAMEBUFFER", ptr,
                                                       encbuf.size());
          if (shared_mem.get()) {
            shared_mem->writeMsg((void *)ptr, encbuf.size());
            shared_mem->notify_all();
          }

          getAttributeCache()->setOutputDomainAsChanged();
        }
      } catch (...) {
        goInFatalError("Encode exception ", -1004, __PRETTY_FUNCTION__);
      }
    } else {
      if (ret == TRIGGER_TIMEOUT_ERROR) {
        RTCameraBaseLERR_ << " wait returned:" << ret;
        setStateVariableSeverity(
            StateVariableTypeAlarmDEV, "capture_timeout",
            chaos::common::alarm::MultiSeverityAlarmLevelWarning);

      } else if (ret < 0) {
        RTCameraBaseLERR_ << " Error wait returned:" << ret;
        setStateVariableSeverity(
            StateVariableTypeAlarmDEV, "capture_error",
            chaos::common::alarm::MultiSeverityAlarmLevelHigh);
      }
    }
  }
}

//! Execute the Control Unit work
void RTCameraBase::unitStop() throw(chaos::CException) {
  isRunning = false;
  stopGrabbing();
}

//! Deinit the Control Unit
void RTCameraBase::unitDeinit() throw(chaos::CException) {
  stopGrabbing();

  captureImg.consume_all([this](buf_t i) {
    free(i.buf);
    captureQueue--;
  });

  encodedImg.consume_all([this](encoded_t i) {
    delete (i.img);
    encodeQueue--;
  });

  driver->cameraDeinit();
}

chaos::common::data::CDWUniquePtr RTCameraBase::getAction(chaos::common::data::CDWUniquePtr p){
  chaos::common::data::CDataWrapper* camera_props=new chaos::common::data::CDataWrapper();
  int ret = 0;
  if ((ret = driver->getCameraProperties(*camera_props)) != 0) {
    RTCameraBaseLERR_ << "Error retriving camera properties ret:" << ret;
  }
  return chaos::common::data::CDWUniquePtr(camera_props);
}
chaos::common::data::CDWUniquePtr RTCameraBase::setAction(chaos::common::data::CDWUniquePtr p){
  return chaos::common::data::CDWUniquePtr();
}
void RTCameraBase::fatalErrorHandler(const chaos::CException &r) {
  stopGrabbing();
}

//! pre imput attribute change
void RTCameraBase::unitInputAttributePreChangeHandler() throw(
    chaos::CException) {}

//! attribute changed handler
void RTCameraBase::unitInputAttributeChangedHandler() throw(chaos::CException) {
  //    std::vector<VariableIndexType> changed;
  //    std::vector<VariableIndexType>::iterator j;
  //    getAttributeCache()->getChangedInputAttributeIndex(changed);
  //    //RTCameraBaseLAPP_<<"UnitRun";

  //    for(j=changed.begin();j!=changed.end();j++){
  //        const char** buffer;

  //        getAttributeCache()->getReadonlyCachedAttributeValue<char>(DOMAIN_INPUT,
  //        *j, &buffer); if(driver->writeChannel(buffer,*j,input_size[*j])){
  //            RTCameraBaseLDBG_<<"writing output channel "<<*j<<", size
  //            :"<<input_size[*j];
  //        }
  //    }
  //    getAttributeCache()->resetChangedInputIndex();
}

/*
CDataWrapper *RTCameraBase::my_custom_action(CDataWrapper *actionParam, bool&
detachParam) { CDataWrapper *result =  new CDataWrapper(); return result;
}
*/
