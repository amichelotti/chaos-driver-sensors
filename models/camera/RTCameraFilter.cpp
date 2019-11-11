/*
 *	RTCameraFilter.cpp
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

#include "RTCameraFilter.h"
#include <chaos/cu_toolkit/control_manager/AttributeSharedCacheWrapper.h>
#include <driver/sensors/core/AbstractSensorDriver.h>
#include <driver/sensors/core/CameraDriverInterface.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace chaos;
using namespace chaos::common::data::cache;
using namespace chaos::cu::driver_manager::driver;
using namespace ::driver::sensor::camera;
using namespace chaos::cu::control_manager;
using namespace cv;
using namespace boost::interprocess;

PUBLISHABLE_CONTROL_UNIT_IMPLEMENTATION(RTCameraFilter)

/*#define RTCameraFilterLAPP_		LAPP_ << "[RTCameraFilter] "
#define RTCameraFilterLDBG_		LDBG_ << "[RTCameraFilter] " <<
__PRETTY_FUNCTION__ << " " #define RTCameraFilterLERR_		LERR_ <<
"[RTCameraFilter] " << __PRETTY_FUNCTION__ << "("<<__LINE__<<") "
*/
#define RTCameraFilterLAPP_ CUAPP
#define RTCameraFilterLDBG_ CUDBG
#define RTCameraFilterLERR_ CUERR
/*
 Construct
 */

RTCameraFilter::RTCameraFilter(
    const string &_control_unit_id, const string &_control_unit_param,
    const ControlUnitDriverList &_control_unit_drivers)
    : RTCameraBase(_control_unit_id, _control_unit_param, _control_unit_drivers,
                   0) {
  RTCameraFilterLDBG_ << "Creating " << _control_unit_id
                      << " params:" << _control_unit_param;
  try {
    chaos::common::data::CDataWrapper p;
    p.setSerializedJsonData(_control_unit_param.c_str());

    apply_moment = false;
    moment_circle = 0;
    REFMOMENTX = -1;
    REFMOMENTY = -1;
    REFMOMENTRADIUS = 0;
    if (p.hasKey(FILTER_MOMENT_KEY)) {
      moment_circle = p.getInt32Value(FILTER_MOMENT_KEY);
      apply_moment = true;
    }
  } catch (...) {
  }
}

/*
 Destructor
 */
RTCameraFilter::~RTCameraFilter() {}

int RTCameraFilter::filtering(cv::Mat &image) {}

#define SETINTPROP(name, n, value)                                             \
  if (name == #n) {                                                            \
    n = value;                                                                 \
    return true;                                                               \
  }
bool RTCameraFilter::setProp(const std::string &name, const std::string &value,
                             uint32_t size) {
  return false;
}

bool RTCameraFilter::setProp(const std::string &name, int32_t value,
                             uint32_t size) {
  int ret;
  int32_t valuer;
  RTCameraFilterLDBG_ << "SET IPROP:" << name << " VALUE:" << value;
  SETINTPROP(name, REFMOMENTX, value);
  SETINTPROP(name, REFMOMENTY, value);
  SETINTPROP(name, REFMOMENTRADIUS, value);

  return RTCameraBase::setProp(name, value, size);
}

bool RTCameraFilter::setProp(const std::string &name, double value,
                             uint32_t size) {
  return RTCameraBase::setProp(name, value, size);
}
//! Return the definition of the control unit
/*!
The api that can be called withi this method are listed into
"Control Unit Definition Public API" module into html documentation
(chaosframework/Documentation/html/group___control___unit___definition___api.html)
*/
void RTCameraFilter::unitDefineActionAndDataset() throw(chaos::CException) {

  addAttributeToDataSet("ROIX", "Source X of Software ROI",
                        chaos::DataType::TYPE_INT32, chaos::DataType::Input);

  addAttributeToDataSet("ROIY", "Source Y of Software ROI",
                        chaos::DataType::TYPE_INT32, chaos::DataType::Input);

  addAttributeToDataSet("ROIXSIZE", "X Size of Software ROI",
                        chaos::DataType::TYPE_INT32, chaos::DataType::Input);

  addAttributeToDataSet("ROIYSIZE", "Y Size of Software ROI",
                        chaos::DataType::TYPE_INT32, chaos::DataType::Input);
  addHandlerOnInputAttributeName<::driver::sensor::camera::RTCameraFilter,
                                 int32_t>(
      this, &::driver::sensor::camera::RTCameraFilter::setProp, "ROIX");

  addHandlerOnInputAttributeName<::driver::sensor::camera::RTCameraFilter,
                                 int32_t>(
      this, &::driver::sensor::camera::RTCameraFilter::setProp, "ROIY");

  addHandlerOnInputAttributeName<::driver::sensor::camera::RTCameraFilter,
                                 int32_t>(
      this, &::driver::sensor::camera::RTCameraFilter::setProp, "ROIXSIZE");

  addHandlerOnInputAttributeName<::driver::sensor::camera::RTCameraFilter,
                                 int32_t>(
      this, &::driver::sensor::camera::RTCameraFilter::setProp, "ROIYSIZE");

  if (apply_moment) {
    addAttributeToDataSet("MOMENTX", "Centroid X", chaos::DataType::TYPE_INT32,
                          chaos::DataType::Output);
    addAttributeToDataSet("MOMENTY", "Centroid Y", chaos::DataType::TYPE_INT32,
                          chaos::DataType::Output);
    addAttributeToDataSet("MOMENTERR", "Distance from reference and current",
                          chaos::DataType::TYPE_DOUBLE,
                          chaos::DataType::Output);

    addAttributeToDataSet("REFMOMENTX", "Ref Centroid X",
                          chaos::DataType::TYPE_INT32, chaos::DataType::Input);
    addAttributeToDataSet("REFMOMENTY", "Ref Centroid Y",
                          chaos::DataType::TYPE_INT32, chaos::DataType::Input);

    addAttributeToDataSet("REFMOMENTRADIUS",
                          "Generate Error if outside this radius",
                          chaos::DataType::TYPE_INT32, chaos::DataType::Input);
    addStateVariable(StateVariableTypeAlarmCU, "moment_out_of_set",
                     "moment out of reference moment radius");
  }
  RTCameraBase::unitDefineActionAndDataset();
}
