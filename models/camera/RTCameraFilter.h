/*
 *	RTCameraFilter.h
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
#ifndef ChaosRTControlUnit_RTCameraFilter_h
#define ChaosRTControlUnit_RTCameraFilter_h
#include <boost/lockfree/queue.hpp>
#include <chaos/cu_toolkit/control_manager/RTAbstractControlUnit.h>
#include <common/misc/data/core/SharedMem.h>
#ifdef CERN_ROOT

class TH2F;
class TF2;

#endif


#include "RTCameraBase.h"
namespace driver {

namespace sensor {
namespace camera {
class CameraDriverInterface;
class RTCameraFilter : public RTCameraBase {
  PUBLISHABLE_CONTROL_UNIT_INTERFACE(RTCameraFilter);

public:
  /*!
   Construct a new CU with full constructor
   */
  RTCameraFilter(const std::string &_control_unit_id,
                 const std::string &_control_unit_param,
                 const ControlUnitDriverList &_control_unit_drivers);
  /*!
   Destructor a new CU
   */
  ~RTCameraFilter();

protected:
  bool apply_roi, apply_gauss_fit;
  int32_t ROIX, ROIY, ROIXSIZE, ROIYSIZE;
  double *REFX, *REFY, *REFSX, *REFSY, *REFRHO;
  int32_t moment_circle;
  bool remove_src;
  /*
      bool setDrvProp(const std::string &name, int32_t value, uint32_t size);
      bool setDrvProp(const std::string &name, double value, uint32_t size);
      bool setDrvProp(const std::string &name, chaos::common::data::CDataWrapper
     value, uint32_t size);
  */
  double Amplitude, X_m, S_x, Y_m, S_y, rho;
#ifdef CERN_ROOT

  TH2F *h;
  TF2 *g2d;
  int gauss_fit_level;
#endif
  /*!
  Define the Control Unit Dataset and Actions
  */
  void unitDefineActionAndDataset() ;

  int filtering(cv::Mat &image);
  void unitInit() ;
};
} // namespace camera
} // namespace sensor
} // namespace driver
#endif
