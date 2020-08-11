/*
 *	BaslerScoutDriver.h
 *	!CHAOS
 *	Created by Andrea Michelotti.
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
#include "BaslerScoutDriver.h"
#include "HardwareTriggerConfiguration.h"
#include <boost/lexical_cast.hpp>
#include <chaos/common/data/CDataWrapper.h>
#include <chaos/cu_toolkit/driver_manager/driver/AbstractDriverPlugin.h>
#include <math.h>
#include <pylon/PylonIncludes.h>
#include <pylon/gige/ActionTriggerConfiguration.h>
#include <stdlib.h>
#include <string>
// use the pylon driver
#include <chaos/common/data/CDataVariant.h>
#include <pylon/ConfigurationEventHandler.h>
#include <pylon/gige/BaslerGigEInstantCamera.h>
using namespace Pylon;
namespace cu_driver = chaos::cu::driver_manager::driver;
using namespace ::driver::sensor::camera;
#define BaslerScoutDriverLAPP_ LAPP_ << "[BaslerScoutDriver] "
#define BaslerScoutDriverLDBG LDBG_ << "[BaslerScoutDriver] "

#define BaslerScoutDriverLDBG_                                                 \
  LDBG_ << "[BaslerScoutDriver (" << serial_dev << "," << friendly_name        \
        << "):" << __FUNCTION__ << "] "
#define BaslerScoutDriverLERR_                                                 \
  LERR_ << "[BaslerScoutDriver (" << serial_dev << "," << friendly_name        \
        << "):" << __PRETTY_FUNCTION__ << "] "
#define BaslerScoutDriverLERR LERR_ << "[BaslerScoutDriver] "
// GET_PLUGIN_CLASS_DEFINITION
// we need only to define the driver because we don't are makeing a plugin
OPEN_CU_DRIVER_PLUGIN_CLASS_DEFINITION(BaslerScoutDriver, 1.0.0, ::driver::sensor::camera::BaslerScoutDriver)
REGISTER_CU_DRIVER_PLUGIN_CLASS_INIT_ATTRIBUTE(::driver::sensor::camera::BaslerScoutDriver, http_address / dnsname: port)
CLOSE_CU_DRIVER_PLUGIN_CLASS_DEFINITION
OPEN_REGISTER_PLUGIN
REGISTER_PLUGIN(::driver::sensor::camera::BaslerScoutDriver)
CLOSE_REGISTER_PLUGIN
#define GETINTVALUE(x, y, LOG)                                                 \
  {                                                                            \
    int32_t val;                                                               \
    LOG << "GETTING INT PROP \"" << #x << "\" alias:" << y;                    \
    if (getNode(#x, (&cam), val) == 0) {                                       \
      p->addInt32Value(y, (int32_t)val);                                       \
    } else {                                                                   \
      BaslerScoutDriverLERR_ << "cannot read basler node \"" << #x << "\"";    \
    }                                                                          \
  }

#define GETINTPERCVALUE(x, y, LOG)                                             \
  {                                                                            \
    LOG << "GETTING INT PERCENT PROP \"" << #x << "\" alias:" << y;            \
    float per;                                                                 \
    if (getNodeInPercentage(#x, (&cam), per) == 0) {                           \
      p->addDoubleValue(y, per);                                               \
    }                                                                          \
  }

#define SETINODE(name, cam, val, ret)                                          \
  if (setNode(name, cam, (int64_t)val) == 0) {                                 \
    BaslerScoutDriverLDBG << "setting \"" << name << "\" = " << val;           \
  } else {                                                                     \
    ret++;                                                                     \
    BaslerScoutDriverLERR << "ERROR setting \"" << name << "\" = " << val;     \
  }

template <typename T> static T Adjust(T val, T minimum, T maximum, T inc) {
  // Check the input parameters.
  if (inc <= 0) {
    // Negative increments are invalid.
    throw LOGICAL_ERROR_EXCEPTION("Unexpected increment %d", inc);
  }
  if (minimum > maximum) {
    // Minimum must not be bigger than or equal to the maximum.
    throw LOGICAL_ERROR_EXCEPTION("minimum bigger than maximum.");
  }

  // Check the lower bound.
  if (val < minimum) {
    return minimum;
  }

  // Check the upper bound.
  if (val > maximum) {
    return maximum;
  }

  // Check the increment.
  if (inc == 1) {
    // Special case: all values are valid.
    return val;
  } else {
    // The value must be min + (n * inc).
    // Due to the integer division, the value will be rounded down.
    return minimum + (((val - minimum) / inc) * inc);
  }
}

void BaslerScoutDriver::driverInit(const char *initParameter) throw(
    chaos::CException) {
  throw chaos::CException(
      -3, "You should provide a valid JSON initialization string",
      __PRETTY_FUNCTION__);
}

void BaslerScoutDriver::driverInit(
    const chaos::common::data::CDataWrapper &json) throw(chaos::CException) {
  BaslerScoutDriverLDBG_ << "Initializing BASLER json driver:"
                         << json.getCompliantJSONString();
  props->reset();
  ownprops->reset();
  props->appendAllElement((chaos::common::data::CDataWrapper &)json);
  parseInitCommonParams(json);
  if (initializeCamera(*props) != 0) {
    throw chaos::CException(
        -1, "cannot initialize camera " + json.getCompliantJSONString(),
        __PRETTY_FUNCTION__);
  }
}

void BaslerScoutDriver::driverDeinit() throw(chaos::CException) {
  BaslerScoutDriverLAPP_ << "Deinit BASLER driver";
}
using namespace GenApi;

namespace driver {

namespace sensor {
namespace camera {

static int setNode(const std::string &node_name, CInstantCamera &camera,
                   int64_t val) {

  try {
    BaslerScoutDriverLDBG << "setting int node:" << node_name << " to " << val;
    INodeMap &control = camera.GetNodeMap();
    GenApi::CIntegerPtr node = control.GetNode(node_name.c_str());
    if (node.IsValid() == false) {
      BaslerScoutDriverLERR << "Node:" << node_name << " is invalid";

      return -1;
    }
    if (IsWritable(node)) {
      node->SetValue(val);
    } else {
      BaslerScoutDriverLERR << "Node:" << node_name << " is not writable";
      return -100;
    }
  } catch (const GenericException &e) {
    // Error handling.
    BaslerScoutDriverLERR << "An exception occurred during set of Node:"
                          << node_name;
    BaslerScoutDriverLERR << e.GetDescription();
    return -3;
  } catch (...) {
    BaslerScoutDriverLERR << "An exception occurre during set of Node:"
                          << node_name;
    return -2;
  }
  return 0;
}

static int setNode(const std::string &node_name, CInstantCamera &camera,
                   double val) {
  try {
    BaslerScoutDriverLDBG << "setting float node:" << node_name
                          << " to:" << val;

    INodeMap &control = camera.GetNodeMap();
    GenApi::CFloatPtr node = control.GetNode(node_name.c_str());
    if (node.IsValid() == false) {
      BaslerScoutDriverLERR << "Node:" << node_name << " is invalid";

      return -1;
    }
    node->SetValue(val);
    /*  if(IsWritable(node)){
        node->SetValue(val);
    } else {
        BaslerScoutDriverLERR_<<  "Node:"<<node_name<< " is not writable";
        return -100;

    }*/
  } catch (const GenericException &e) {
    // Error handling.
    BaslerScoutDriverLERR << "An exception occurred during SET of Node:"
                          << node_name;
    BaslerScoutDriverLERR << e.GetDescription();
    return -3;
  } catch (...) {
    BaslerScoutDriverLERR << "An exception occurre during SET of Node:"
                          << node_name;
    return -2;
  }
  return 0;
  return 0;
}
static int setNodeInPercentage(const std::string &node_name,
                               CInstantCamera &camera, float percent) {
  try {
    BaslerScoutDriverLDBG << "setting int node:" << node_name
                          << " to: " << percent * 100 << " %";

    INodeMap &control = camera.GetNodeMap();
    GenApi::CIntegerPtr node = control.GetNode(node_name.c_str());
    if (node.IsValid() == false) {
      BaslerScoutDriverLERR << "Node:" << node_name << " is invalid";

      return -1;
    }
    int64_t min, max, inc;
    min = node->GetMin();
    max = node->GetMax();
    inc = node->GetInc();
    int64_t newGainRaw = min + ((max - min) * percent);
    // Make sure the calculated value is valid
    BaslerScoutDriverLDBG << "MIN:" << min << " MAX:" << max << " INC:" << inc
                          << " VALUE:" << newGainRaw
                          << " percent:" << percent * 100;

    newGainRaw = Adjust(newGainRaw, min, max, inc);
    node->SetValue(newGainRaw);
  } catch (const GenericException &e) {
    // Error handling.
    BaslerScoutDriverLERR << "An exception occurred during SET of Node:"
                          << node_name;
    BaslerScoutDriverLERR << e.GetDescription();
    return -3;
  } catch (...) {
    BaslerScoutDriverLERR << "An exception occurre during SET of Node:"
                          << node_name;
    return -2;
  }
  return 0;
}

int BaslerScoutDriver::getNode(const std::string &node_name,
                               CInstantCamera *camera, int32_t &percent,
                               const std::string pub) {
  try {
    BaslerScoutDriverLDBG_ << "getting node:" << node_name;

    INodeMap &control = camera->GetNodeMap();
    GenApi::CIntegerPtr node = control.GetNode(node_name.c_str());
    if (node.IsValid() == false) {
      BaslerScoutDriverLERR_ << "Node:" << node_name << " is invalid";

      return -1;
    }
    percent = -1;
    percent = node->GetValue();
    /*  if(ownprops->hasKey(node_name)){
            ownprops->setProperty(node_name,node->GetValue());
        } else*/ {
      ownprops->createProperty(node_name, node->GetValue(), node->GetMin(),
                               node->GetMax(), node->GetInc(), pub);
    }
    BaslerScoutDriverLDBG_ << "VAL:" << percent;
  } catch (const GenericException &e) {
    // Error handling.
    BaslerScoutDriverLERR_ << "An exception occurred during GET of Node:\""
                           << node_name << "\":" << e.GetDescription();
    return -3;
  } catch (...) {
    BaslerScoutDriverLERR_ << "An exception occurre during GET of Node:"
                           << node_name;
    return -2;
  }
  return 0;
}
int BaslerScoutDriver::getNodeInPercentage(const std::string &node_name,
                                           CInstantCamera *camera,
                                           float &percent,
                                           const std::string &pub) {
  int64_t min, max, inc, val;
  try {
    BaslerScoutDriverLDBG_ << "getting node:" << node_name;

    INodeMap &control = camera->GetNodeMap();
    GenApi::CIntegerPtr node = control.GetNode(node_name.c_str());
    if (node.IsValid() == false) {
      BaslerScoutDriverLERR_ << "Node:" << node_name << " is invalid";

      return -1;
    }
    percent = -1;
    min = node->GetMin();
    max = node->GetMax();
    inc = node->GetInc();
    val = node->GetValue();
    /*if(ownprops->hasKey(node_name)){
            ownprops->setProperty(node_name,node->GetValue());
        } else*/ {
      ownprops->createProperty(node_name, node->GetValue(), node->GetMin(),
                               node->GetMax(), node->GetInc(), pub);
    }
    if ((max - min) > 0) {
      percent = (val * 1.0 - min) * 100.0 / (float)(max - min);
    }
    BaslerScoutDriverLDBG_ << "VAL:" << val << "MIN:" << min << " MAX:" << max
                           << " PERCENT:" << percent;
  } catch (const GenericException &e) {
    // Error handling.
    BaslerScoutDriverLERR_ << "An exception occurred during GET of Node:"
                           << node_name;
    BaslerScoutDriverLERR_ << e.GetDescription();
    return -3;
  } catch (...) {
    BaslerScoutDriverLERR_ << "An exception occurre during GET of Node:"
                           << node_name;
    return -2;
  }
  return 0;
}
/*
static void setNodeInPercentage(const GenApi::CFloatPtr& node,float percent){
    double min,max,inc;
    if(node.get()==NULL){
        BaslerScoutDriverLERR_<<"node not present";
        return;
    }
    min=node->GetMin() ;
    max=node->GetMax();
    inc=node->GetInc();
    BaslerScoutDriverLDBG_<<"MIN:"<<min<<" MAX:"<<max<<" INC:"<<inc;
    double newGainRaw = min+ ((max - min)* percent);
    // Make sure the calculated value is valid
    newGainRaw = Adjust(newGainRaw,  min, max, inc);

    node->SetValue(newGainRaw);
}
*/
class CConfigurationEvent : public CConfigurationEventHandler {
  ::driver::sensor::camera::BaslerScoutDriver *driver;

public:
  CConfigurationEvent(::driver::sensor::camera::BaslerScoutDriver *drv)
      : driver(drv){};

  void OnAttach(CInstantCamera & /*camera*/) {
    BaslerScoutDriverLDBG << "OnAttach event";
  }

  void OnAttached(CInstantCamera &camera) {
    BaslerScoutDriverLDBG << "OnAttached event for device "
                          << camera.GetDeviceInfo().GetModelName();
  }

  void OnOpen(CInstantCamera &camera) {
    BaslerScoutDriverLDBG << "OnOpen event for device "
                          << camera.GetDeviceInfo().GetModelName();
  }

  void OnOpened(CInstantCamera &camera) { driver->propsToCamera(camera, NULL); }

  void OnGrabStart(CInstantCamera &camera) {
    BaslerScoutDriverLDBG << "OnGrabStart event for device "
                          << camera.GetDeviceInfo().GetModelName();
  }

  void OnGrabStarted(CInstantCamera &camera) {
    BaslerScoutDriverLDBG << "OnGrabStarted event for device "
                          << camera.GetDeviceInfo().GetModelName();
  }

  void OnGrabStop(CInstantCamera &camera) {
    BaslerScoutDriverLDBG << "OnGrabStop event for device "
                          << camera.GetDeviceInfo().GetModelName();
  }

  void OnGrabStopped(CInstantCamera &camera) {
    BaslerScoutDriverLDBG << "OnGrabStopped event for device "
                          << camera.GetDeviceInfo().GetModelName();
  }

  void OnClose(CInstantCamera &camera) {
    BaslerScoutDriverLDBG << "OnClose event for device "
                          << camera.GetDeviceInfo().GetModelName();
  }

  void OnClosed(CInstantCamera &camera) {
    BaslerScoutDriverLDBG << "OnClosed event for device "
                          << camera.GetDeviceInfo().GetModelName();
  }

  void OnDestroy(CInstantCamera &camera) {
    BaslerScoutDriverLDBG << "OnDestroy event for device "
                          << camera.GetDeviceInfo().GetModelName();
  }

  void OnDestroyed(CInstantCamera & /*camera*/) {
    BaslerScoutDriverLDBG << "OnDestroyed event";
  }

  void OnDetach(CInstantCamera &camera) {
    BaslerScoutDriverLDBG << "OnDetach event for device "
                          << camera.GetDeviceInfo().GetModelName();
  }

  void OnDetached(CInstantCamera &camera) {
    BaslerScoutDriverLDBG << "OnDetached event for device "
                          << camera.GetDeviceInfo().GetModelName();
  }

  void OnGrabError(CInstantCamera &camera, const String_t errorMessage) {
    BaslerScoutDriverLDBG << "OnGrabError event for device "
                          << camera.GetDeviceInfo().GetModelName();
    BaslerScoutDriverLDBG << "Error Message: " << errorMessage;
  }

  void OnCameraDeviceRemoved(CInstantCamera &camera) {
    BaslerScoutDriverLDBG << "OnCameraDeviceRemoved event for device "
                          << camera.GetDeviceInfo().GetModelName();
  }
};

class CCameraEvent : public CCameraEventHandler {
public:
  virtual void OnCameraEvent(CInstantCamera &camera, intptr_t userProvidedId,
                             GenApi::INode *pNode) {
    BaslerScoutDriverLDBG << "OnCameraEvent event for device "
                          << camera.GetDeviceInfo().GetModelName();
    BaslerScoutDriverLDBG << "User provided ID: " << userProvidedId;
    BaslerScoutDriverLDBG << "Event data node name: " << pNode->GetName();
    GenApi::CValuePtr ptrValue(pNode);
    if (ptrValue.IsValid()) {
      BaslerScoutDriverLDBG << "Event node data: " << ptrValue->ToString();
    }
  }
};
} // namespace camera
} // namespace sensor
} // namespace driver
// GET_PLUGIN_CLASS_DEFINITION
// we need to define the driver with alias version and a class that implement it
int BaslerScoutDriver::initializeCamera(
    const chaos::common::data::CDataWrapper &json) {
  IPylonDevice *pdev = NULL;
  int found = 0;
  try {
    if (camerap) {
      BaslerScoutDriverLDBG_ << "Deleting camera before";

      camerap->Close();
      delete camerap;
      camerap = NULL;
    }
    if (camerap == NULL) {
      PylonInitialize();
      BaslerScoutDriverLDBG_ << "Pylon driver initialized";
      // Create an instant camera object for the camera device found first.
      BaslerScoutDriverLDBG_ << "getting  camera informations ..";
      if (!serial.empty()) {
        CTlFactory &tlFactory = CTlFactory::GetInstance();
        DeviceInfoList_t devices;
        // Get all attached devices and exit application if no device is found.
        if (tlFactory.EnumerateDevices(devices) == 0) {
          BaslerScoutDriverLERR_ << " No camera present!!!";
          throw chaos::CException(-1,
                                  "Cannot initialize camera " +
                                      json.getCompliantJSONString() +
                                      ": No camera present!",
                                  __PRETTY_FUNCTION__);

          return -1;
        }
        CInstantCameraArray cameras(devices.size());
        BaslerScoutDriverLDBG_ << "Found " << cameras.GetSize()
                               << " cameras, looking for serial:" << serial;

        for (size_t i = 0; (i < cameras.GetSize()) && (found == 0); ++i) {
          pdev = tlFactory.CreateDevice(devices[i]);

          cameras[i].Attach(pdev);

          BaslerScoutDriverLDBG_
              << i << "] Model:" << cameras[i].GetDeviceInfo().GetModelName();
          BaslerScoutDriverLDBG_
              << i << "] Friendly Name: "
              << cameras[i].GetDeviceInfo().GetFriendlyName();
          BaslerScoutDriverLDBG_ << i << "] Full Name    : "
                                 << cameras[i].GetDeviceInfo().GetFullName();

          BaslerScoutDriverLDBG_
              << i << "] SerialNumber : "
              << cameras[i].GetDeviceInfo().GetSerialNumber();

          if (serial == cameras[i].GetDeviceInfo().GetSerialNumber().c_str()) {

            BaslerScoutDriverLDBG_
                << " FOUND " << cameras[i].GetDeviceInfo().GetSerialNumber();
            serial_dev = serial;
            friendly_name = cameras[i].GetDeviceInfo().GetFriendlyName();

            cameras[i].DetachDevice();
            camerap = new CInstantCamera(pdev);

            found++;
            /* if (camerap && (!camerap->IsOpen()))
             {
                 BaslerScoutDriverLDBG_ << "Opening Camera";

                 camerap->Open();
             }*/
          } else {
            // BaslerScoutDriverLDBG_ << " removing " <<
            // cameras[i].GetDeviceInfo().GetSerialNumber();
            cameras[i].DetachDevice();
            tlFactory.DestroyDevice(pdev);
            // delete pdev;
          }
        }
      } else {
        BaslerScoutDriverLDBG_
            << "No \"serial\" specified getting first camera..";
        pdev = CTlFactory::GetInstance().CreateFirstDevice();
        camerap = new CInstantCamera(pdev);
        BaslerScoutDriverLDBG_ << " Model:"
                               << camerap->GetDeviceInfo().GetModelName();
        BaslerScoutDriverLDBG_ << "Friendly Name: "
                               << camerap->GetDeviceInfo().GetFriendlyName();
        BaslerScoutDriverLDBG_ << "Full Name    : "
                               << camerap->GetDeviceInfo().GetFullName();
        BaslerScoutDriverLDBG_ << "SerialNumber : "
                               << camerap->GetDeviceInfo().GetSerialNumber();
        serial_dev = camerap->GetDeviceInfo().GetSerialNumber();
        friendly_name = camerap->GetDeviceInfo().GetFriendlyName();
      }
      if (camerap) {
        std::string model_name =
            camerap->GetDeviceInfo().GetModelName().c_str();
        std::string friendname =
            camerap->GetDeviceInfo().GetFriendlyName().c_str();
        std::string fullname = camerap->GetDeviceInfo().GetFullName().c_str();
        std::string sn = camerap->GetDeviceInfo().GetSerialNumber().c_str();
        ownprops->createProperty("Model", (const std::string &)model_name);
        ownprops->createProperty("Friend", (const std::string &)friendname);
        ownprops->createProperty("FullName", (const std::string &)fullname);
        ownprops->createProperty("SerialNumber", (const std::string &)sn);
      }
      if (camerap && json.hasKey("TRIGGER_MODE")) {
        int tmode = json.getInt32Value("TRIGGER_MODE");

        changeTriggerMode(camerap, tmode);
      }
    }
    if (camerap) {
      if ((!camerap->IsOpen())) {
        BaslerScoutDriverLDBG_ << "Opening Camera";

        camerap->Open();
      }
      propsToCamera(*camerap, (chaos::common::data::CDataWrapper *)&json);

      return 0;
    }
  } catch (const GenericException &e) {
    BaslerScoutDriverLERR_ << "Cannot attach camera "
                           << json.getCompliantJSONString()
                           << " error:" << e.GetDescription();
    throw chaos::CException(-1,
                            "Cannot attach camera " +
                                json.getCompliantJSONString() + ":" +
                                e.GetDescription(),
                            __PRETTY_FUNCTION__);

  } catch (...) {
    BaslerScoutDriverLERR_ << " exception attaching device:"
                           << json.getCompliantJSONString();
  }
  return -1;
}
/*
BaslerScoutDriver::BaslerScoutDriver():camerap(NULL),shots(0),framebuf(NULL),fn(NULL),props(NULL),tmode(CAMERA_TRIGGER_CONTINOUS),gstrategy(CAMERA_LATEST_ONLY){

    BaslerScoutDriverLDBG_<<  "Created BASLER Driver";
    props=new chaos::common::data::CDataWrapper();


}
*/
#define STOP_GRABBING(camera)                                                  \
  if (camera == NULL) {                                                        \
    BaslerScoutDriverLERR_ << "Invalid Camera";                                \
    return -1;                                                                 \
  }                                                                            \
  if (!stopGrabbing) {                                                         \
    stopGrab();                                                                \
    restore_grab = true;                                                       \
  }

#define RESTORE_GRABBING(camera)                                               \
  if (restore_grab) {                                                          \
    startGrab(shots, framebuf, fn);                                            \
  }

DEFAULT_CU_DRIVER_PLUGIN_CONSTRUCTOR_WITH_NS(::driver::sensor::camera,
                                             BaslerScoutDriver) {
  camerap = NULL;
  shots = 0;
  framebuf = NULL;
  fn = NULL;
  tmode = CAMERA_TRIGGER_CONTINOUS;
  gstrategy = CAMERA_LATEST_ONLY;

  BaslerScoutDriverLDBG_ << "Created BASLER Driver";
  triggerHWSource = "Line1";
  stopGrabbing = true;
  restore_grab = false;
}

// default descrutcor
BaslerScoutDriver::~BaslerScoutDriver() {
  if (camerap) {
    BaslerScoutDriverLDBG_ << "deleting camera";

    delete camerap;
    camerap = NULL;
  }
}
int BaslerScoutDriver::changeTriggerMode(Pylon::CInstantCamera *camera,
                                         int trigger_mode) {
  try {
    STOP_GRABBING(camera);
    camera->Close();
    switch (trigger_mode) {
    case (CAMERA_TRIGGER_CONTINOUS): {
      BaslerScoutDriverLDBG_ << " TRIGGER CONTINOUS";

      camerap->RegisterConfiguration(new CAcquireContinuousConfiguration,
                                     RegistrationMode_ReplaceAll,
                                     Cleanup_Delete);
      break;
    }
    case CAMERA_TRIGGER_SINGLE: {
      BaslerScoutDriverLDBG_ << " TRIGGER SINGLE";

      camerap->RegisterConfiguration(new CAcquireSingleFrameConfiguration,
                                     RegistrationMode_ReplaceAll,
                                     Cleanup_Delete);
      break;
    }
    case CAMERA_TRIGGER_SOFT: {
      BaslerScoutDriverLDBG_ << " TRIGGER SOFT";

      camerap->RegisterConfiguration(new CSoftwareTriggerConfiguration,
                                     RegistrationMode_ReplaceAll,
                                     Cleanup_Delete);

      break;
    }
    case CAMERA_TRIGGER_HW_LOW: {
      BaslerScoutDriverLDBG_ << " TRIGGER LOW Edge of " << triggerHWSource;
      CHardwareTriggerConfiguration::setHWTrigger(triggerHWSource,
                                                  "FallingEdge");
      // camerap->RegisterConfiguration(new CAcquireSingleFrameConfiguration,
      // RegistrationMode_ReplaceAll, Cleanup_Delete);
      camerap->RegisterConfiguration(new CHardwareTriggerConfiguration,
                                     RegistrationMode_ReplaceAll,
                                     Cleanup_Delete);
      break;
    }
    case CAMERA_TRIGGER_HW_HI:

    {
      BaslerScoutDriverLDBG_ << " TRIGGER HI Edge of " << triggerHWSource;
      CHardwareTriggerConfiguration::setHWTrigger(triggerHWSource,
                                                  "RisingEdge");
      // camerap->RegisterConfiguration(new CAcquireSingleFrameConfiguration,
      // RegistrationMode_ReplaceAll, Cleanup_Delete);
      camerap->RegisterConfiguration(new CHardwareTriggerConfiguration,
                                     RegistrationMode_ReplaceAll,
                                     Cleanup_Delete);
      break;
    }
    }
    tmode = (TriggerModes)trigger_mode;
      camera->Open();
    RESTORE_GRABBING(camera);
  } catch (const GenericException &e) {
    std::stringstream ss;
    ss << "Cannot change trigger mode to " << trigger_mode
       << " error:" << e.GetDescription() << " in:" << e.GetSourceFileName();
    LERR_ << ss.str();
    camera->Open();

    RESTORE_GRABBING(camera);

    return -1;
    // throw chaos::CException(-1, "Cannot change trigger " + ss.str(),
    // __PRETTY_FUNCTION__);

  } catch (...) {
    LERR_ << "Uknown exception";
    RESTORE_GRABBING(camera);

    return -2;
  }
  return 0;
}
static Pylon::EPixelType cv2basler(const std::string &fmt) {
  if (fmt == "CV_8UC1")
    return Pylon::EPixelType::PixelType_Mono8;
  if (fmt == "CV_8SC1")
    return Pylon::EPixelType::PixelType_Mono8signed;
  if (fmt == "CV_16UC1")
    return Pylon::EPixelType::PixelType_Mono16;
  if (fmt == "CV_8UC3")
    return Pylon::EPixelType::PixelType_RGB8packed;
  return Pylon::EPixelType::PixelType_Mono8;
}
static std::string basler2cv(Pylon::EPixelType fmt) {
  switch (fmt) {
  case Pylon::EPixelType::PixelType_BayerBG16: {
    return "BAYERBG16";
  }
  case Pylon::EPixelType::PixelType_BayerGB16: {
    return "BAYERGB16";
  }
  case Pylon::EPixelType::PixelType_BayerGR16: {
    return "BAYERGR16";
  }
  case Pylon::EPixelType::PixelType_BayerRG16: {
    return "BAYERRG16";
  }
  case Pylon::EPixelType::PixelType_Mono8:
    return "CV_8UC1";
  case Pylon::EPixelType::PixelType_Mono8signed:
    return "CV_8SC1";
  case Pylon::EPixelType::PixelType_Mono16:
    return "CV_16UC1";
  case Pylon::EPixelType::PixelType_RGB8packed:
    return "CV_8UC3";
  default: { return "NOT SUPPORTED"; }
  }
}

int BaslerScoutDriver::cameraToProps(Pylon::CInstantCamera &cam,
                                     chaos::common::data::CDataWrapper *p) {
  using namespace GenApi;
  int32_t ivalue;
  if (p == NULL) {
    BaslerScoutDriverLERR_ << "Invalid Parameter";
    return -1;
  }
  try {
    BaslerScoutDriverLDBG_ << "Updating Camera Properties";
    INodeMap &nodemap = cam.GetNodeMap();
    CEnumerationPtr pixelFormat(nodemap.GetNode("PixelFormat"));
    if (pixelFormat.IsValid()) {
      //   CPixelTypeMapper pixelTypeMapper(pixelFormat);
      //  EPixelType pixelType =
      //  pixelTypeMapper.GetPylonPixelTypeFromNodeValue(pixelFormat->GetIntValue());

      std::string cv = basler2cv((Pylon::EPixelType)pixelFormat->GetIntValue());

      if (cv == "NOT SUPPORTED") {
        p->addStringValue(FRAMEBUFFER_ENCODING_KEY,
                          pixelFormat->ToString().c_str());

      } else {
        p->addStringValue(FRAMEBUFFER_ENCODING_KEY, cv);
      }
      BaslerScoutDriverLDBG_ << "Pixel format " << cv
                             << " Native:" << pixelFormat->ToString();
    }

    if (getNode("TriggerMode", &cam, ivalue) == 0) {
      BaslerScoutDriverLDBG_ << "GETTING PROP TRIGGER_MODE";

      if (ivalue == Basler_GigECamera::TriggerModeEnums::TriggerMode_On) {
        if (getNode("TriggerSource", &cam, ivalue) == 0) {
          switch (ivalue) {
          case Basler_GigECamera::TriggerSourceEnums::TriggerSource_Software:
            p->addInt32Value("TRIGGER_MODE", CAMERA_TRIGGER_SOFT);
            BaslerScoutDriverLDBG_ << "TRIGGER_MODE SOFT";

            break;
          default: {
            if (getNode("TriggerActivation", &cam, ivalue) == 0) {
              if (ivalue == Basler_GigECamera::TriggerActivationEnums::
                                TriggerActivation_RisingEdge) {
                p->addInt32Value("TRIGGER_MODE", CAMERA_TRIGGER_HW_HI);
                BaslerScoutDriverLDBG_ << "TRIGGER_MODE HW HI";
              } else {
                p->addInt32Value("TRIGGER_MODE", CAMERA_TRIGGER_HW_LOW);
                BaslerScoutDriverLDBG_ << "TRIGGER_MODE HW LO";
              }
            }
          }
          }
        }
      } else {
        p->addInt32Value("TRIGGER_MODE", CAMERA_TRIGGER_CONTINOUS);
        BaslerScoutDriverLDBG_ << "TRIGGER_MODE OFF";
      }
    } else {
      p->addInt32Value("TRIGGER_MODE", tmode);
    }
  } catch (const GenericException &e) {
    // Error handling.
    BaslerScoutDriverLERR_
        << "An exception occurred during update properties, err:"
        << e.GetDescription() << " in:" << e.GetSourceFileName();
    return -2;
  } catch (...) {
    BaslerScoutDriverLERR_ << "An Uknown exception occurred.";
    return -3;
  }
  GETINTVALUE(Width, "WIDTH", BaslerScoutDriverLDBG_);
  GETINTVALUE(Height, "HEIGHT", BaslerScoutDriverLDBG_);
  GETINTVALUE(OffsetX, "OFFSETX", BaslerScoutDriverLDBG_);
  GETINTVALUE(OffsetY, "OFFSETY", BaslerScoutDriverLDBG_);
  GETINTPERCVALUE(GainRaw, "GAIN", BaslerScoutDriverLDBG_);
  GETINTPERCVALUE(ExposureTimeRaw, "SHUTTER", BaslerScoutDriverLDBG_);
  GETINTPERCVALUE(SharpnessEnhancementRaw, "SHARPNESS", BaslerScoutDriverLDBG_);
  GETINTPERCVALUE(BslBrightnessRaw, "BRIGHTNESS", BaslerScoutDriverLDBG_);
  GETINTPERCVALUE(BslContrastRaw, "CONTRAST", BaslerScoutDriverLDBG_);
  p->addCSDataValue("custom", ownprops->getProperty());
  return 0;
}

int BaslerScoutDriver::propsToCamera(CInstantCamera &camera,
                                     chaos::common::data::CDataWrapper *p) {
  // Get the camera control object.
  int ret = 0;
  if (p == NULL) {
    BaslerScoutDriverLDBG_ << "Invalid Parameter";
    return -1;
  }
  BaslerScoutDriverLDBG_ << "setting props: " << p->getCompliantJSONString();

  // Get the parameters for setting the image area of interest (Image AOI).

  // Maximize the Image AOI.
  //   chaos::common::data::CDataWrapper*p=driver->props;
  if (p->hasKey("TRIGGER_MODE")) {
    int value = p->getInt32Value("TRIGGER_MODE");
    BaslerScoutDriverLDBG_ << "setting TRIGGER_MODE: " << value;
    changeTriggerMode(&camera, value);
    /* switch (value)
     {
     case CAMERA_TRIGGER_HW_HI:
         SETINODE("TriggerMode", camera,
     Basler_GigECamera::TriggerModeEnums::TriggerMode_On, ret);
         SETINODE("TriggerSource", camera,
     Basler_GigECamera::TriggerSourceEnums::TriggerSource_VInput1, ret);
         SETINODE("TriggerActivation", camera,
     Basler_GigECamera::TriggerActivationEnums::TriggerActivation_RisingEdge,
     ret);

         break;
     case CAMERA_TRIGGER_HW_LOW:
         SETINODE("TriggerMode", camera,
     Basler_GigECamera::TriggerModeEnums::TriggerMode_On, ret);
         SETINODE("TriggerSource", camera,
     Basler_GigECamera::TriggerSourceEnums::TriggerSource_VInput1, ret);
         SETINODE("TriggerActivation", camera,
     Basler_GigECamera::TriggerActivationEnums::TriggerActivation_FallingEdge,
     ret); break; case CAMERA_TRIGGER_SOFT: SETINODE("TriggerMode", camera,
     Basler_GigECamera::TriggerModeEnums::TriggerMode_On, ret);
         SETINODE("TriggerSource", camera,
     Basler_GigECamera::TriggerSourceEnums::TriggerSource_Software, ret); break;
     case CAMERA_TRIGGER_CONTINOUS:
         SETINODE("TriggerMode", camera,
     Basler_GigECamera::TriggerModeEnums::TriggerMode_Off, ret);

         break;
     }
     */
  }
  STOP_GRABBING(&camera);
  if (p->hasKey("OFFSETX")) {
    SETINODE("OffsetX", camera, p->getInt32Value("OFFSETX"), ret);
  }
  if (p->hasKey("OFFSETY")) {
    SETINODE("OffsetY", camera, p->getInt32Value("OFFSETY"), ret);
  }
  if (p->hasKey("WIDTH")) {
    SETINODE("Width", camera, p->getInt32Value("WIDTH"), ret);
  }
  if (p->hasKey("HEIGHT")) {
    SETINODE("Height", camera, p->getInt32Value("HEIGHT"), ret);
  }
  if (p->hasKey("PIXELFMT")) {
    // Set the pixel data format.
    INodeMap &control = camera.GetNodeMap();

    CEnumerationPtr(control.GetNode("PixelFormat"))
        ->FromString(p->getCStringValue("PIXELFMT"));
    BaslerScoutDriverLDBG_ << "setting Pixel Format "
                           << p->getCStringValue("PIXELFMT");
  }
  if (p->hasKey("GAIN")) {
    double gain = p->getAsRealValue("GAIN") / 100.0;

    if (gain < 0) {
      if (setNode(
              "GainAuto", camera,
              (int64_t)Basler_GigECamera::GainAutoEnums::GainAuto_Continuous) !=
          0) {
        ret++;
      }
    } else {
      // setNode("GainAuto",camera,(int64_t)Basler_GigECamera::GainAutoEnums::GainAuto_Off);

      if (gain > 1.0) {
        gain = 1.0;
      }
      if (setNodeInPercentage("GainRaw", camera, gain) != 0) {
        ret++;
        BaslerScoutDriverLERR_ << "Setting GAIN FAILED:" << gain;
      }
    }
  }
  if (p->hasKey("SHUTTER")) {
    double gain = p->getAsRealValue("SHUTTER") / 100.0;

    if ((gain < 0)) {
      SETINODE("ExposureAuto", camera,
               Basler_GigECamera::ExposureAutoEnums::ExposureAuto_Continuous,
               ret);

    } else {
      SETINODE("ExposureAuto", camera,
               Basler_GigECamera::ExposureAutoEnums::ExposureAuto_Off, ret);

      if (gain > 1.0) {
        gain = 1.0;
      }
      if (setNodeInPercentage("ExposureTimeRaw", camera, gain) != 0) {
        ret++;
        BaslerScoutDriverLERR_ << "Setting SHUTTER FAILED";
      }
    }
  }
  if (p->hasKey("SHARPNESS")) {
    double gain = p->getAsRealValue("SHARPNESS") / 100.0;

    if (gain < 0) {
      SETINODE(
          "DemosaicingMode", camera,
          Basler_GigECamera::DemosaicingModeEnums::DemosaicingMode_BaslerPGI,
          ret);

    } else {
      if (gain > 1.0) {
        gain = 1.0;
      }
      if (setNodeInPercentage("SharpnessEnhancementRaw", camera, gain) != 0) {
        ret++;
        BaslerScoutDriverLERR_ << "Setting SHARPNESS FAILED";
      }
    }
  }
  if (p->hasKey("BRIGHTNESS")) {
    SETINODE("DemosaicingMode", camera,
             Basler_GigECamera::DemosaicingModeEnums::DemosaicingMode_BaslerPGI,
             ret);

    double gain = p->getAsRealValue("BRIGHTNESS") / 100.0;

    if (gain >= 0) {
      if (gain > 1.0) {
        gain = 1.0;
      }
      if (setNodeInPercentage("BslBrightnessRaw", camera, gain) != 0) {
        ret++;
        BaslerScoutDriverLERR_ << "Setting BRIGHTNESS FAILED";
      }
    }
  }
  if (p->hasKey("CONTRAST")) {
    SETINODE("DemosaicingMode", camera,
             Basler_GigECamera::DemosaicingModeEnums::DemosaicingMode_BaslerPGI,
             ret);
    double gain = p->getDoubleValue("CONTRAST") / 100.0;

    if (gain >= 0) {
      if (gain > 1.0) {
        gain = 1.0;
      }
      if (setNodeInPercentage("BslContrastRaw", camera, gain) != 0) {
        ret++;
        BaslerScoutDriverLERR_ << "Setting CONTRAST FAILED";
      }

      //                  BaslerScoutDriverLDBG_ << "CONTRAST "<<gain<<"     : "
      //                  << contrast_node->GetValue() << " (Min: " <<
      //                  contrast_node->GetMin() << "; Max: " <<
      //                  contrast_node->GetMax() << "; Inc: "
      //                  <<contrast_node->GetInc() << ")";
    }
  }
  //         p->addInt32Value("WIDTH", (int32_t)width->GetValue());
  //       p->addInt32Value("HEIGHT", (int32_t)height->GetValue());
  RESTORE_GRABBING(&camera);
  return ret;
}

int BaslerScoutDriver::cameraInit(void *buffer, uint32_t sizeb) {
  BaslerScoutDriverLDBG_ << "Initialization";
  // simple initialization
  if (camerap == NULL) {
    BaslerScoutDriverLERR_ << "no camera available";
    return -1;
  }
  try {
    if (props->hasKey("TRIGGER_MODE")) {
      tmode = (TriggerModes)props->getInt32Value("TRIGGER_MODE");
      if (changeTriggerMode(camerap, tmode) != 0) {
        BaslerScoutDriverLERR_ << "Failed change Trigger to " << tmode;
      }
    }

    // Register the standard configuration event handler for enabling software
    // triggering. The software trigger configuration handler replaces the
    // default configuration as all currently registered configuration handlers
    // are removed by setting the registration mode to
    // RegistrationMode_ReplaceAll.

    BaslerScoutDriverLDBG_ << "trigger configuration handler installed";

    // For demonstration purposes only, add sample configuration event handlers
    // to print out information about camera use and image grabbing.
    camerap->RegisterConfiguration(new CConfigurationEvent(this),
                                   RegistrationMode_Append, Cleanup_Delete);
    BaslerScoutDriverLDBG_ << "event  handler installed";

    // camera->RegisterImageEventHandler( new CCameraEventPrinter,
    // RegistrationMode_Append, Cleanup_Delete);
    // Print the model name of the camera.
    BaslerScoutDriverLDBG_ << "Using device "
                           << camerap->GetDeviceInfo().GetModelName();

    // The MaxNumBuffer parameter can be used to control the count of buffers
    // allocated for grabbing. The default value of this parameter is 10.
    camerap->MaxNumBuffer = 15;
    BaslerScoutDriverLDBG_ << "Open camera";
    if (!camerap->IsOpen()) {
      camerap->Open();
    }

    setNode(
        "DemosaicingMode", *camerap,
        (int64_t)
            Basler_GigECamera::DemosaicingModeEnums::DemosaicingMode_BaslerPGI);
  } catch (const GenericException &e) {
    // Error handling.
    BaslerScoutDriverLERR_ << "An exception occurred. err:"
                           << e.GetDescription()
                           << " in:" << e.GetSourceFileName();
    return -2;
  } catch (...) {
    BaslerScoutDriverLERR_ << "An Uknown exception occurred.";
    return -3;
  }

  return 0;
}

int BaslerScoutDriver::cameraDeinit() {
  BaslerScoutDriverLDBG_ << "deinit";
  if (camerap) {
    camerap->Close();
  }
  return 0;
}

int BaslerScoutDriver::startGrab(uint32_t _shots, void *_framebuf,
                                 cameraGrabCallBack _fn) {
  BaslerScoutDriverLDBG_ << "Start Grabbing";
  shots = _shots;
  framebuf = _framebuf;
  fn = _fn;
  EGrabStrategy strategy;
  if (props->hasKey("GRAB_STRATEGY")) {
    gstrategy = (GrabStrategy)props->getInt32Value("GRAB_STRATEGY");
  }
  switch (gstrategy) {
  case CAMERA_ONE_BY_ONE:
    strategy = GrabStrategy_OneByOne;
    break;
  case CAMERA_LATEST_ONLY:
    strategy = GrabStrategy_LatestImageOnly;
    break;
  case CAMERA_LATEST:
    strategy = GrabStrategy_LatestImages;
    break;
  case CAMERA_INCOMING:
    strategy = GrabStrategy_UpcomingImage;

    break;
  }
  if (shots > 0) {
    camerap->StartGrabbing((size_t)shots, strategy);
  } else {
    camerap->StartGrabbing(strategy);
  }
  stopGrabbing = false;
  return 0;
}
int BaslerScoutDriver::waitGrab(const char **img, uint32_t timeout_ms) {
  if (camerap == NULL) {
    BaslerScoutDriverLERR_ << "Invalid Camera ";

    return -1;
  }
  if (stopGrabbing) {
    BaslerScoutDriverLERR_ << "Grabbing is stopped ";
    return -2;
  }
  Pylon::CGrabResultPtr ptrGrabResult;
  if (tmode > CAMERA_TRIGGER_SINGLE) {
    if (camerap->WaitForFrameTriggerReady(timeout_ms,
                                          TimeoutHandling_ThrowException)) {
      if (tmode == CAMERA_TRIGGER_SOFT) {
        camerap->ExecuteSoftwareTrigger();
      }
    } else {
      BaslerScoutDriverLERR_ << "TRIGGER TIMEOUT : ";

      return TRIGGER_TIMEOUT_ERROR;
    }

    while ((camerap->GetGrabResultWaitObject().Wait(0) == 0) &&
           (stopGrabbing == false)) {
      WaitObject::Sleep(1);
    }
  }
  int nBuffersInQueue = 0;
  if (camerap->RetrieveResult(timeout_ms, ptrGrabResult,
                              TimeoutHandling_Return)) {
    if (ptrGrabResult->GetNumberOfSkippedImages()) {
      BaslerScoutDriverLDBG_ << "Skipped "
                             << ptrGrabResult->GetNumberOfSkippedImages()
                             << " image, timeout ms" << timeout_ms;
    }
    if (ptrGrabResult->GrabSucceeded()) {
      // Access the image data.

      const uint8_t *pImageBuffer = (uint8_t *)ptrGrabResult->GetBuffer();
      //      cout << "Gray value of first pixel: " << (uint32_t)
      //      pImageBuffer[0] << endl << endl;

      // CPylonImage target;
      // CImageFormatConverter converter;
      // converter.OutputPixelFormat=PixelType_RGB8packed;
      // converter.OutputPixelFormat=PixelType_BGR8packed;
      //  converter.OutputPixelFormat=PixelType_Mono8;
      //  converter.OutputBitAlignment=OutputBitAlignment_MsbAligned;
      // converter.Convert(target,ptrGrabResult);

      // int
      // size_ret=(bcount<target.GetImageSize())?bcount:target.GetImageSize();
      //            memcpy(buffer,target.GetBuffer(),size_ret);

      //             int size_ret=(bcount<=
      //             ptrGrabResult->GetImageSize())?bcount:
      //             ptrGrabResult->GetImageSize();
      int size_ret = ptrGrabResult->GetImageSize();
      BaslerScoutDriverLDBG_ << " Size " << ptrGrabResult->GetWidth() << "x"
                             << ptrGrabResult->GetHeight()
                             << " Image Raw Size: " << size_ret;
      if (img) {
        *img = (const char *)pImageBuffer;
        // memcpy(hostbuf,pImageBuffer,size_ret);
      }
      if (fn) {
        fn(pImageBuffer, size_ret, ptrGrabResult->GetWidth(),
           ptrGrabResult->GetHeight());
      }

      return size_ret;
    } else {
      BaslerScoutDriverLERR_ << "Error: " << ptrGrabResult->GetErrorCode()
                             << " " << ptrGrabResult->GetErrorDescription();
      return CAMERA_GRAB_ERROR;
    }
  }
  /*
    while (camerap->RetrieveResult(timeout_ms, ptrGrabResult,
    TimeoutHandling_Return))
    {
        if (ptrGrabResult->GetNumberOfSkippedImages())
        {
            BaslerScoutDriverLDBG_ << "Skipped " <<
    ptrGrabResult->GetNumberOfSkippedImages() << " image, timeout
    ms"<<timeout_ms;
        }
        if (ptrGrabResult->GrabSucceeded())
        {
            // Access the image data.

            const uint8_t *pImageBuffer = (uint8_t *)ptrGrabResult->GetBuffer();
            //      cout << "Gray value of first pixel: " << (uint32_t)
    pImageBuffer[0] << endl << endl;

            // CPylonImage target;
            // CImageFormatConverter converter;
            // converter.OutputPixelFormat=PixelType_RGB8packed;
            // converter.OutputPixelFormat=PixelType_BGR8packed;
            //  converter.OutputPixelFormat=PixelType_Mono8;
            //  converter.OutputBitAlignment=OutputBitAlignment_MsbAligned;
            //converter.Convert(target,ptrGrabResult);

            // int
    size_ret=(bcount<target.GetImageSize())?bcount:target.GetImageSize();
            //            memcpy(buffer,target.GetBuffer(),size_ret);

            //             int size_ret=(bcount<=
    ptrGrabResult->GetImageSize())?bcount: ptrGrabResult->GetImageSize(); int
    size_ret = ptrGrabResult->GetImageSize(); BaslerScoutDriverLDBG_ << " Size "
    << ptrGrabResult->GetWidth() << "x" << ptrGrabResult->GetHeight() << " Image
    Raw Size: " << size_ret; if (img)
            {
                *img = (const char *)pImageBuffer;
                //memcpy(hostbuf,pImageBuffer,size_ret);
            }
            if (fn)
            {
                fn(pImageBuffer, size_ret, ptrGrabResult->GetWidth(),
    ptrGrabResult->GetHeight());
            }

            return size_ret;
        }
        else
        {
            BaslerScoutDriverLERR_ << "Error: " << ptrGrabResult->GetErrorCode()
    << " " << ptrGrabResult->GetErrorDescription(); return CAMERA_GRAB_ERROR;
        }

        nBuffersInQueue++;
    }
     BaslerScoutDriverLDBG_ << "Retrieved " << nBuffersInQueue << " grab results
    from output queue.";
*/

  return 0;
}

int BaslerScoutDriver::waitGrab(uint32_t timeout_ms) {
  return waitGrab((const char **)&framebuf, timeout_ms);
}
int BaslerScoutDriver::stopGrab() {
  BaslerScoutDriverLDBG_ << "Stop  Grabbing";

  stopGrabbing = true;
  camerap->StopGrabbing();
  return 0;
}

int BaslerScoutDriver::setImageProperties(int32_t width, int32_t height,
                                          int32_t opencvImageType) {
  props->setValue("WIDTH", width);
  props->setValue("HEIGHT", width);
  if (camerap) {
    return propsToCamera(*camerap, props.get());
  }
  return -1;
}

int BaslerScoutDriver::getImageProperties(int32_t &width, int32_t &height,
                                          int32_t &opencvImageType) {
  ChaosUniquePtr<chaos::common::data::CDataWrapper> cw(
      new chaos::common::data::CDataWrapper());
  int ret = -1;
  cameraToProps(*camerap, cw.get());
  if (cw->hasKey("WIDTH")) {
    width = cw->getInt32Value("WIDTH");
    ret = 0;
  }

  if (cw->hasKey("HEIGHT")) {
    height = cw->getInt32Value("HEIGHT");
    ret++;
  }

  return (ret > 0) ? 0 : ret;
}

int BaslerScoutDriver::setCameraProperty(const std::string &propname,
                                         int32_t val) {
  ChaosUniquePtr<chaos::common::data::CDataWrapper> cw(
      new chaos::common::data::CDataWrapper());
  BaslerScoutDriverLDBG_ << "Setting Int\"" << propname << "\"=" << (int32_t)val;

  cw->addInt32Value(propname, val);
  int ret=propsToCamera(*camerap, cw.get());
  return ret;
}
int BaslerScoutDriver::setCameraProperty(const std::string &propname,
                                         double val) {
  ChaosUniquePtr<chaos::common::data::CDataWrapper> cw(
      new chaos::common::data::CDataWrapper());
  BaslerScoutDriverLDBG_ << "Setting double \"" << propname << "\"=" << val;

  cw->addDoubleValue(propname, val);
  int ret=propsToCamera(*camerap, cw.get());
  return ret;
}

int BaslerScoutDriver::getCameraProperty(const std::string &propname,
                                         int32_t &val) {
  ChaosUniquePtr<chaos::common::data::CDataWrapper> cw(
      new chaos::common::data::CDataWrapper());

  cameraToProps(*camerap, cw.get());

  if (cw->hasKey(propname)) {
    val = cw->getInt32Value(propname);
    BaslerScoutDriverLDBG_ << "Property Int \"" << propname << "\"=" << val;

    return 0;
  }
  return -1;
}

int BaslerScoutDriver::getCameraProperty(const std::string &propname,
                                         double &val) {
  ChaosUniquePtr<chaos::common::data::CDataWrapper> cw(
      new chaos::common::data::CDataWrapper());

  cameraToProps(*camerap, cw.get());

  if (cw->hasKey(propname)) {
    val = cw->getDoubleValue(propname);
    BaslerScoutDriverLDBG_ << "Property Double \"" << propname << "\"=" << val;

    return 0;
  }
  return -1;
}

int BaslerScoutDriver::getCameraProperties(
    chaos::common::data::CDataWrapper &proplist) {
  int ret = cameraToProps(*camerap, &proplist);
  BaslerScoutDriverLDBG_ << "PROPERTIES:" << proplist.getCompliantJSONString()
                         << " ret:" << ret;

  return ret;
}
