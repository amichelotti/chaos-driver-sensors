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
#include  <boost/lockfree/queue.hpp> 
#include <common/misc/data/core/SharedMem.h>
#include <chaos/cu_toolkit/control_manager/RTAbstractControlUnit.h>
#ifdef CERN_ROOT
#include "TASImage.h"
#include "TCanvas.h"

#endif

#define DEFAULT_RESOLUTION 640*480*3

#include "RTCameraBase.h"
namespace driver{
    
    namespace sensor{
         namespace camera{
	   class CameraDriverInterface;
class RTCameraFilter : public  RTCameraBase{
	PUBLISHABLE_CONTROL_UNIT_INTERFACE(RTCameraFilter);


public:
    /*!
     Construct a new CU with full constructor
     */
    RTCameraFilter(const std::string& _control_unit_id, const std::string& _control_unit_param, const ControlUnitDriverList& _control_unit_drivers);
    /*!
     Destructor a new CU
     */
    ~RTCameraFilter();

protected:
        bool apply_roi,apply_moment,apply_gauss_fit;
        int32_t ROIX,ROIY,ROIXSIZE,ROIYSIZE,REFMOMENTX,REFMOMENTY,REFMOMENTRADIUS;
        int32_t moment_circle;
        bool remove_src;
        bool setProp(const std::string &name, int32_t value, uint32_t size);
        bool setProp(const std::string &name, double value, uint32_t size);
        bool setProp(const std::string &name, chaos::common::data::CDataWrapper value, uint32_t size);


		/*!
		Define the Control Unit Dataset and Actions
		*/
		void unitDefineActionAndDataset() throw(chaos::CException);
	
    int filtering(cv::Mat&image);


};
    }}}
#endif
