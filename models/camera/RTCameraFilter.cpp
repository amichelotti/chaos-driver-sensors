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
                   0),remove_src(false)
{
  RTCameraFilterLDBG_ << "Creating " << _control_unit_id
                      << " params:" << _control_unit_param;
  try
  {
    chaos::common::data::CDataWrapper p;
    p.setSerializedJsonData(_control_unit_param.c_str());

    apply_moment = false;
    moment_circle = 0;
    REFMOMENTX = -1;
    REFMOMENTY = -1;
    REFMOMENTRADIUS = 0;
    if (p.hasKey(FILTER_MOMENT_KEY))
    {
      moment_circle = p.getInt32Value(FILTER_MOMENT_KEY);
      apply_moment = true;
    }
  }
  catch (...)
  {
  }
}

/*
 Destructor
 */
RTCameraFilter::~RTCameraFilter() {}

#define SETINTPROP(name, n, value) \
  if (name == #n)                  \
  {                                \
    n = value;                     \
    return true;                   \
  }
bool RTCameraFilter::setProp(const std::string &name,  chaos::common::data::CDataWrapper p,
                             uint32_t size)
{

  if (name != "FILTER")
  {
    return false;
  }
  
  if(p.hasKey(FILTER_MOMENT_KEY)&&p.isInt32Value(FILTER_MOMENT_KEY)){
      moment_circle = p.getInt32Value(FILTER_MOMENT_KEY);
      apply_moment = true;
    } else {
      apply_moment = true;
    }
    if(p.hasKey(FILTER_REMOVE_SOURCE_KEY)&&p.isBoolValue(FILTER_REMOVE_SOURCE_KEY)){
      remove_src=p.getBoolValue(FILTER_REMOVE_SOURCE_KEY);
    }
  
 
  return true;
}

bool RTCameraFilter::setProp(const std::string &name, int32_t value,
                             uint32_t size)
{
  int ret;
  int32_t valuer;
  RTCameraFilterLDBG_ << "SET IPROP:" << name << " VALUE:" << value;
  SETINTPROP(name, REFMOMENTX, value);
  SETINTPROP(name, REFMOMENTY, value);
  SETINTPROP(name, REFMOMENTRADIUS, value);

  return RTCameraBase::setProp(name, value, size);
}

bool RTCameraFilter::setProp(const std::string &name, double value,
                             uint32_t size)
{
  return RTCameraBase::setProp(name, value, size);
}
//! Return the definition of the control unit
/*!
The api that can be called withi this method are listed into
"Control Unit Definition Public API" module into html documentation
(chaosframework/Documentation/html/group___control___unit___definition___api.html)
*/
void RTCameraFilter::unitDefineActionAndDataset() throw(chaos::CException)
{

  addAttributeToDataSet("FILTER", "Filters to apply (JSON)",
                        chaos::DataType::TYPE_CLUSTER, chaos::DataType::Input);

  addHandlerOnInputAttributeName<::driver::sensor::camera::RTCameraFilter,
                                  chaos::common::data::CDataWrapper>(
      this, &::driver::sensor::camera::RTCameraFilter::setProp, std::string("FILTER"));

  if (apply_moment)
  {
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
int RTCameraFilter::filtering(cv::Mat &image)
{
  if (apply_moment)
  {
    Mat thr, gray;

    // convert image to grayscale
    cvtColor(image, gray, COLOR_BGR2GRAY);

    // convert grayscale to binary image
    threshold(gray, thr, 100, 255, THRESH_BINARY);

    // find moments of the image
    Moments m = moments(thr, true);
    Point p(m.m10 / m.m00, m.m01 / m.m00);
    getAttributeCache()->setOutputAttributeValue("MOMENTX", (void *)&p.x, sizeof(int32_t));
    getAttributeCache()->setOutputAttributeValue("MOMENTY", (void *)&p.y, sizeof(int32_t));

    if (moment_circle > 0)
    {
      circle(image, p, moment_circle, Scalar(128, 0, 0), -1);
    }
    if ((REFMOMENTX > 0) && (REFMOMENTY > 0))
    {
      double dist = sqrt(pow((p.x - REFMOMENTX), 2) + pow((p.y - REFMOMENTY), 2));
      getAttributeCache()->setOutputAttributeValue("MOMENTERR", (void *)&dist, sizeof(dist));
      if (REFMOMENTRADIUS > 0)
      {
        if (dist > REFMOMENTRADIUS)
        {
          circle(image, Point(REFMOMENTX, REFMOMENTY), REFMOMENTRADIUS, Scalar(128, 0, 0), 1);

          setStateVariableSeverity(StateVariableTypeAlarmCU, "moment_out_of_set",
                                   chaos::common::alarm::MultiSeverityAlarmLevelWarning);
        }
        else
        {
          circle(image, Point(REFMOMENTX, REFMOMENTY), REFMOMENTRADIUS, Scalar(0, 255, 0), 1);

          setStateVariableSeverity(StateVariableTypeAlarmCU, "moment_out_of_set",
                                   chaos::common::alarm::MultiSeverityAlarmLevelClear);
        }
      }
    }
  }

  return 0;
}
