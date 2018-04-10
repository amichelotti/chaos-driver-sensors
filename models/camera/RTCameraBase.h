/*
 *	RTCameraBase.h
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
#ifndef ChaosRTControlUnit_RTCameraBase_h
#define ChaosRTControlUnit_RTCameraBase_h

#include <chaos/cu_toolkit/control_manager/RTAbstractControlUnit.h>

#define DEFAULT_RESOLUTION 640*480*3

namespace driver{
    
    namespace sensor{
         namespace camera{
	   class CameraDriverInterface;
class RTCameraBase : public chaos::cu::control_manager::RTAbstractControlUnit {
	PUBLISHABLE_CONTROL_UNIT_INTERFACE(RTCameraBase);


public:
    /*!
     Construct a new CU with full constructor
     */
    RTCameraBase(const std::string& _control_unit_id, const std::string& _control_unit_param, const ControlUnitDriverList& _control_unit_drivers);
    /*!
     Destructor a new CU
     */
    ~RTCameraBase();

protected:
    int32_t *sizex,*sizey;
        const int32_t* mode;
        uint8_t* framebuf;
        uint8_t* framebuf_out;
        char encoding[16];
        chaos::common::data::CDataWrapper camera_props;

        double*shutter,*brightness,*contrast,*sharpness,*gain;
         char*fmt;
        CameraDriverInterface*driver;
        bool setProp(const std::string &name, int32_t value, uint32_t size);
        bool setProp(const std::string &name, double value, uint32_t size);

		/*!
		Define the Control Unit Dataset and Actions
		*/
		void unitDefineActionAndDataset()throw(chaos::CException);
		/*!(Optional)
		Define the Control Unit custom attribute
		*/
		void unitDefineCustomAttribute();
    /*!(Optional)
     Initialize the Control Unit and all driver, with received param from MetadataServer
     */
		void unitInit() throw(chaos::CException);
    /*!(Optional)
     Execute the work, this is called with a determinated delay
     */
    void unitStart() throw(chaos::CException);
    /*!
     Execute the work, this is called with a determinated delay, it must be as fast as possible
     */
    void unitRun() throw(chaos::CException);

    /*!(Optional)
     The Control Unit will be stopped
     */
    void unitStop() throw(chaos::CException);

    /*!(Optional)
     The Control Unit will be deinitialized and disposed
     */
    void unitDeinit() throw(chaos::CException);

		//! Pre input attribute change
		/*!(Optional)
		This handler is called befor the update of the
		cached input attribute with the requested valure
		*/
		void unitInputAttributePreChangeHandler() throw(chaos::CException);

		//! Handler called on the update of one or more input attribute
		/*!(Optional)
		After an input attribute has been chacnged this handler
		is called
		*/
		void unitInputAttributeChangedHandler() throw(chaos::CException);


};
    }}}
#endif
