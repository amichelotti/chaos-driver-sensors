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
    switch(cvenc){case CV_8UC4:case CV_8SC4:bpp=4;break;\
    case CV_8UC3:case CV_8SC3:bpp=3;break;\
    case CV_8UC2:case CV_8SC2:bpp=2;break;\
    case CV_8UC1:case CV_8SC1:bpp=1;break;\
    case CV_16UC1:case CV_16SC1:bpp=2;break;\
    case CV_16UC2:case CV_16SC2:bpp=4;break;\
    case CV_16UC3:case CV_16SC3:bpp=6;break;\
    case CV_32SC1:bpp=4;break;\
    case CV_32SC2:bpp=8;break;\
    case CV_32SC3:bpp=16;break;\
    default:bpp=16;}}
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
#define RTCameraBaseLAPP_ CUAPP
#define RTCameraBaseLDBG_ CUDBG
#define RTCameraBaseLERR_ CUERR
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
      imagesizey(0), apply_resize(false), trigger_timeout(0), bpp(3) {
  RTCameraBaseLDBG_ << "Creating " << _control_unit_id
                    << " params:" << _control_unit_param;

  framebuf = NULL;
  try {
    chaos::common::data::CDataWrapper p;
    p.setSerializedJsonData(_control_unit_param.c_str());
    if (p.hasKey(FRAMEBUFFER_ENCODING_KEY) &&
        p.isStringValue(FRAMEBUFFER_ENCODING_KEY)) {
      std::string enc = p.getStringValue(FRAMEBUFFER_ENCODING_KEY);
      framebuf_encoding = fmt2cv(enc);
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
}
void RTCameraBase::updateProperty() {
  std::vector<std::string> props;
  if (driver->getCameraProperties(camera_props) != 0) {
    throw chaos::CException(-1, "Error retrieving camera properties",
                            __PRETTY_FUNCTION__);
  }
  camera_props.getAllKey(props);
  AttributeSharedCacheWrapper *cc = getAttributeCache();

  for (std::vector<std::string>::iterator i = props.begin(); i != props.end();
       i++) {
    if (!camera_props.isCDataWrapperValue(*i)) {

      if (camera_props.getValueType(*i) == chaos::DataType::TYPE_DOUBLE) {
        double tmp, *p;
        driver->getCameraProperty(*i, tmp);
        RTCameraBaseLDBG_ << "Camera Property double " << *i
                          << " VALUE:" << tmp;

        p = cc->getRWPtr<double>(DOMAIN_OUTPUT, *i);
        *p = tmp;
      }
      if (camera_props.getValueType(*i) == chaos::DataType::TYPE_INT32) {
        int32_t tmp, *p;
        driver->getCameraProperty(*i, tmp);
        RTCameraBaseLDBG_ << "Camera Property int " << *i << " VALUE:" << tmp;

        p = cc->getRWPtr<int32_t>(DOMAIN_OUTPUT, *i);
        *p = tmp;
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

bool RTCameraBase::setProp(const std::string &name, int32_t value,
                           uint32_t size) {
  int ret;
  int32_t valuer;
  RTCameraBaseLDBG_ << "SET IPROP:" << name << " VALUE:" << value;

  bool stopgrab = (stopCapture == false) &&
                  (((name == WIDTH_KEY) && (value != *sizex)) ||
                   ((name == HEIGHT_KEY) && (value != *sizey)) ||
                   ((name == OFFSETX_KEY) && (value != *offsetx)) ||
                   ((name == OFFSETY_KEY) && (value != *offsety)));
  if (stopgrab) {
    stopGrabbing();
  }
  ret = driver->setCameraProperty(name, value);
  setStateVariableSeverity(StateVariableTypeAlarmDEV, "operation_not_supported",
                           chaos::common::alarm::MultiSeverityAlarmLevelClear);

  if (ret != 0) {
    std::stringstream ss;
    ss << "error setting \"" << name << "\" to " << value;
    setStateVariableSeverity(
        StateVariableTypeAlarmDEV, "operation_not_supported",
        chaos::common::alarm::MultiSeverityAlarmLevelWarning);
    metadataLogging(
        chaos::common::metadata_logging::StandardLoggingChannel::LogLevelError,
        ss.str());
  }

  // updateProperty();
  driver->getCameraProperty(name, valuer);
  AttributeSharedCacheWrapper *cc = getAttributeCache();
  int32_t *p = cc->getRWPtr<int32_t>(DOMAIN_OUTPUT, name);
  *p = valuer;
  RTCameraBaseLDBG_ << "SET IPROP:" << name << " SET VALUE:" << value
                    << " READ VALUE:" << valuer << " ret:" << ret;
  if (stopgrab) {
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
  setStateVariableSeverity(StateVariableTypeAlarmDEV, "operation_not_supported",
                           chaos::common::alarm::MultiSeverityAlarmLevelClear);

  if (ret != 0) {
    setStateVariableSeverity(
        StateVariableTypeAlarmDEV, "operation_not_supported",
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
  chaos::cu::driver_manager::driver::DriverAccessor *acc =
      getAccessoInstanceByIndex(0);
  if (acc == NULL) {
    throw chaos::CException(-1, "No associated camera driver",
                            __PRETTY_FUNCTION__);
  }
  driver = new CameraDriverInterface(acc);
  if (driver == NULL) {
    throw chaos::CException(-1, "cannot create driver", __PRETTY_FUNCTION__);
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
                        chaos::DataType::Bidirectional);
  if (buffering > 1) {
    addAttributeToDataSet("CAPTURE_FRAMERATE", "Capture Frame Rate",
                          chaos::DataType::TYPE_INT32, chaos::DataType::Output);
    addAttributeToDataSet("ENCODE_FRAMERATE", "Encode Frame Rate",
                          chaos::DataType::TYPE_INT32, chaos::DataType::Output);
  }

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
      StateVariableTypeAlarmDEV, "operation_not_supported",
      "the operation in not supported i.e property not present or accesible");
  addStateVariable(StateVariableTypeAlarmCU, "encode_error",
                   "an error occurred during encode");
  addStateVariable(StateVariableTypeAlarmDEV, "capture_error",
                   "an error occurred during capture");
  addStateVariable(StateVariableTypeAlarmDEV, "capture_timeout",
                   "a timeout has occurred");

  addStateVariable(StateVariableTypeAlarmDEV, "captureQueueFull",
                   "Queue Capture Full");
  addStateVariable(StateVariableTypeAlarmDEV, "encodeQueueFull",
                   "Queue Encode Full");
}

//! Define custom control unit attribute
void RTCameraBase::unitDefineCustomAttribute() {
  std::string config;
  chaos::common::data::CDataWrapper attr;
  if (driver == NULL) {
    chaos::cu::driver_manager::driver::DriverAccessor *acc =
        getAccessoInstanceByIndex(0);
    if (acc == NULL) {
      throw chaos::CException(-1, "No associated camera driver",
                              __PRETTY_FUNCTION__);
    }
    driver = new CameraDriverInterface(acc);
    if (driver == NULL) {
      throw chaos::CException(-1, "cannot create driver", __PRETTY_FUNCTION__);
    }
  }
  if (driver->getCameraProperties(camera_props) != 0) {
    throw chaos::CException(-1, "Error retrieving camera properties",
                            __PRETTY_FUNCTION__);
  }
  getAttributeCache()->addCustomAttribute("config", 8192,
                                          chaos::DataType::TYPE_CLUSTER);
  if (driver->getCameraProperties(attr) != 0) {
    throw chaos::CException(-1, "Error retrieving camera properties",
                            __PRETTY_FUNCTION__);
  }
  config = attr.getCompliantJSONString();
  RTCameraBaseLDBG_ << "ADDING CONFIG:" << config;
  getAttributeCache()->setCustomAttributeValue("config", (void *)config.c_str(),
                                               config.size());
  getAttributeCache()->setCustomDomainAsChanged();
  pushCustomDataset();
}

//! Initialize the Custom Control Unit
void RTCameraBase::unitInit() throw(chaos::CException) {
  int ret;
  int32_t itype;
  int32_t width, height;
  AttributeSharedCacheWrapper *cc = getAttributeCache();
  // this properties must exist
  //
  sizex = cc->getRWPtr<int32_t>(DOMAIN_INPUT, WIDTH_KEY);
  sizey = cc->getRWPtr<int32_t>(DOMAIN_INPUT, HEIGHT_KEY);
  offsetx = cc->getRWPtr<int32_t>(DOMAIN_INPUT, OFFSETX_KEY);
  offsety = cc->getRWPtr<int32_t>(DOMAIN_INPUT, OFFSETY_KEY);
  //
  mode = cc->getROPtr<int32_t>(DOMAIN_INPUT, TRIGGER_MODE_KEY);
  //   camera_out=cc->getRWPtr<uint8_t>(DOMAIN_OUTPUT, "FRAMEBUFFER");
  fmt = cc->getRWPtr<char>(DOMAIN_INPUT, "FMT");
  ofmt = cc->getRWPtr<char>(DOMAIN_OUTPUT, "FMT");
  if (buffering > 1) {
    capture_frame_rate =
        cc->getRWPtr<int32_t>(DOMAIN_OUTPUT, "CAPTURE_FRAMERATE");
    enc_frame_rate = cc->getRWPtr<int32_t>(DOMAIN_OUTPUT, "ENCODE_FRAMERATE");
  }
  std::vector<std::string> props;
  camera_props.getAllKey(props);
  // breanch number and soft reset

  if ((ret = driver->cameraInit(0, 0)) != 0) {
    throw chaos::CException(ret, "cannot initialize camera",
                            __PRETTY_FUNCTION__);
  }

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

  if (driver->getImageProperties(width, height, itype) == 0) {
    RTCameraBaseLDBG_ << "CAMERA IMAGE:" << width << "x" << height;
    *sizex = width;
    *sizey = height;
  }
  chaos::common::data::CDataWrapper cam_prop;
  if (driver->getCameraProperties(cam_prop) == 0) {
    if (cam_prop.hasKey(FRAMEBUFFER_ENCODING_KEY) &&
        cam_prop.isStringValue(FRAMEBUFFER_ENCODING_KEY)) {
      std::string enc = cam_prop.getStringValue(FRAMEBUFFER_ENCODING_KEY);
      DECODE_CVENCODING(enc, CV_8UC4);
      DECODE_CVENCODING(enc, CV_8UC3);
      DECODE_CVENCODING(enc, CV_8UC2);
      DECODE_CVENCODING(enc, CV_8UC1);
      DECODE_CVENCODING(enc, CV_8SC1);
      DECODE_CVENCODING(enc, CV_8SC2);
      DECODE_CVENCODING(enc, CV_8SC3);
      DECODE_CVENCODING(enc, CV_8SC4);

      DECODE_CVENCODING(enc, CV_16UC4);
      DECODE_CVENCODING(enc, CV_16UC3);
      DECODE_CVENCODING(enc, CV_16UC2);
      DECODE_CVENCODING(enc, CV_16UC1);
      DECODE_CVENCODING(enc, CV_16SC1);
      DECODE_CVENCODING(enc, CV_16SC2);
      DECODE_CVENCODING(enc, CV_16SC3);
      DECODE_CVENCODING(enc, CV_16SC4);
      DECODE_CVENCODING(enc, CV_32SC1);
      DECODE_CVENCODING(enc, CV_32SC2);
      DECODE_CVENCODING(enc, CV_32SC3);
      DECODE_CVENCODING(enc, CV_32SC4);
    }
  }
  if ((*sizex == 0) || (*sizey == 0)) {
    RTCameraBaseLERR_ << "Invalid image sizes:" << *sizex << "x" << *sizey;
    throw chaos::CException(-3, "Invalid image size!!", __PRETTY_FUNCTION__);
  }
  if (!apply_resize) {
    imagesizex = *sizex;
    imagesizey = *sizey;
  }
  cc->setOutputAttributeNewSize("FRAMEBUFFER", imagesizex * imagesizey * (bpp),
                                true);

  // framebuf = (uint8_t*)malloc(size);
  RTCameraBaseLDBG_ << "Starting acquiring imagex " << *sizex << "x" << *sizey
                    << " bpp:" << bpp;
  if (*fmt == 0) {
    strcpy(encoding, ".png");

  } else {
    snprintf(encoding, sizeof(encoding), ".%s", fmt);
  }
  strncpy(ofmt, encoding, sizeof(encoding));
  getAttributeCache()->setInputDomainAsChanged();
  getAttributeCache()->setOutputDomainAsChanged();
}
void RTCameraBase::cameraGrabCallBack(const void *buf, uint32_t blen,
                                      uint32_t width, uint32_t heigth,
                                      uint32_t error) {}
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
  RTCameraBaseLDBG_ << "Allocated " << buffering << " buffers of "
                    << (*sizex * *sizey * bpp) << " bytes";

  try {
    if(shared_mem.get()==NULL){
      shared_mem.reset(new ::common::misc::data::SharedMem(
          getCUID(), (imagesizex * imagesizey * bpp)));
      RTCameraBaseLDBG_ << "opened sharedMem \"" << shared_mem->getName()
                        << "\" at @" << std::hex << shared_mem->getMem()
                        << " size:" << std::dec << shared_mem->getSize();
    } else {
      shared_mem->resize(imagesizex * imagesizey * bpp);
    }
    } catch (boost::interprocess::interprocess_exception &ex) {
    LERR_ << "##cannot open shared memory :" << ex.what();
  }

  driver->startGrab(0);
  stopCapture = false;
  if (buffering > 1) {
    capture_th = boost::thread(&RTCameraBase::captureThread, this);
  }
  getAttributeCache()->setCustomDomainAsChanged();
  pushCustomDataset();
}

void RTCameraBase::stopGrabbing() {
  RTCameraBaseLDBG_ << "Stop Grabbing..." << stopCapture;

  driver->stopGrab();

  if (buffering > 1) {
 
  stopCapture = true;

  metadataLogging(
      chaos::common::metadata_logging::StandardLoggingChannel::LogLevelInfo,
      "Stop grabbing");

      wait_capture.notify_all();
    wait_encode.notify_all();
    full_encode.notify_all();
    full_capture.notify_all();
    capture_th.join();
    encode_th.join();
  }
        RTCameraBaseLDBG_ << "Stop Grabbing done" << stopCapture;

  /*    for(int cnt=0;cnt<CAMERA_FRAME_BUFFERING;cnt++){
        free(framebuf_out[cnt].buf);
        framebuf_out[cnt].size=0;
        framebuf_out[cnt].buf=NULL;
    }*/

  setStateVariableSeverity(StateVariableTypeAlarmDEV, "operation_not_supported",
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
void RTCameraBase::unitStart() throw(chaos::CException) { startGrabbing(); }

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

      if (ret == TRIGGER_TIMEOUT_ERROR) {
        RTCameraBaseLERR_ << " wait returned:" << ret
                          << " timeout stopped:" << stopCapture;
        setStateVariableSeverity(
            StateVariableTypeAlarmDEV, "capture_timeout",
            chaos::common::alarm::MultiSeverityAlarmLevelWarning);

      } else if (ret < 0) {
        RTCameraBaseLERR_ << " Error wait returned:" << ret;
        setStateVariableSeverity(
            StateVariableTypeAlarmDEV, "capture_error",
            chaos::common::alarm::MultiSeverityAlarmLevelHigh);
        usleep(100000);
      }
    }
  }
  RTCameraBaseLDBG_ << "Capture thread exiting Queue: " << captureQueue;

  captureImg.consume_all([this](buf_t i) {
    free(i.buf);
    captureQueue--;
  });

  RTCameraBaseLDBG_ << "Capture thread ENDED Queue:" << captureQueue;
}
void RTCameraBase::encodeThread() {
  uint64_t start;
  counter_encode = 0;
  RTCameraBaseLDBG_ << "Encode thread STARTED";
  while (!stopCapture) {

    buf_t a;

    if (captureImg.pop(a) && a.buf) {
      start = chaos::common::utility::TimingUtil::getTimeStampInMicroseconds();
      captureQueue--;

      full_capture.notify_one();
      cv::Mat image(*sizey, *sizex, framebuf_encoding, a.buf);

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

        bool code = cv::imencode(encoding, image, *encbuf);
        free(a.buf);
        image.deallocate();
        if (code == false) {
          setStateVariableSeverity(
              StateVariableTypeAlarmCU, "encode_error",
              chaos::common::alarm::MultiSeverityAlarmLevelHigh);
          LERR_ << "Encode error:" << fmt;
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
                                << " Waiting//";

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
      } catch (...) {
        setStateVariableSeverity(
            StateVariableTypeAlarmCU, "encode_error",
            chaos::common::alarm::MultiSeverityAlarmLevelHigh);
        LERR_ << "Encode exception:" << fmt;
        if (encbuf) {
          delete encbuf;
        }
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
  RTCameraBaseLDBG_ << "Encode thread exiting Queue: " << encodeQueue;

  encodedImg.consume_all([this](encoded_t i) {
    delete (i.img);
    encodeQueue--;
  });

  RTCameraBaseLDBG_ << "Encode thread ENDED Queue: " << encodeQueue;
}

 int RTCameraBase::filtering(cv::Mat&image){
   return 0;
}

//! Execute the Control Unit work
void RTCameraBase::unitRun() throw(chaos::CException) {
  uchar *ptr;
  const char *img = 0;

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

      getAttributeCache()->setOutputDomainAsChanged();
      full_encode.notify_one();
      delete (a);

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
      cv::Mat image(*sizey, *sizex, framebuf_encoding, (uint8_t*)img);
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
          LERR_ << "Encode error:" << fmt;
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
        setStateVariableSeverity(
            StateVariableTypeAlarmCU, "encode_error",
            chaos::common::alarm::MultiSeverityAlarmLevelHigh);
        LERR_ << "Encode exception:" << fmt;
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
  stopGrabbing(); 
  }

//! Deinit the Control Unit
void RTCameraBase::unitDeinit() throw(chaos::CException) {

  driver->cameraDeinit();
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
