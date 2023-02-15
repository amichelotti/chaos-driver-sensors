/*
 *	SCUTA.h
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
#ifndef __SCUTA__
#define __SCUTA__

#include <chaos/cu_toolkit/control_manager/SCAbstractControlUnit.h>
#include "SensorDriverInterface.h"
#include <common/debug/core/debug.h>
#define UTA_DRIVERS 2
#define UTA_TERMOMETER_DRIVER 0
#define UTA_SHUTTER_DRIVER 1

namespace driver {
	namespace sensor {
		
	  enum Temometers{
	    TEMP_INPUT,
	    TEMP_OUTPUT
	  };

	  enum Shutter{
	    SHUTTER_INPUT,
	    SHUTTER_OUTPUT
	  };
	  

		class SCUTA : public chaos::cu::control_manager::SCAbstractControlUnit {
			PUBLISHABLE_CONTROL_UNIT_INTERFACE(SCUTA)

			SensorDriverInterface *driver[UTA_DRIVERS];

		protected:
			/*
			 Define the Control Unit Dataset and Actions
			 */
			void unitDefineActionAndDataset();

			void unitDefineCustomAttribute();
			
			/*(Optional)
			 Initialize the Control Unit and all driver, with received param from MetadataServer
			 */
			void unitInit() ;
			/*(Optional)
			 Execute the work, this is called with a determinated delay, it must be as fast as possible
			 */
			void unitStart() ;
			/*(Optional)
			 The Control Unit will be stopped
			 */
			void unitStop() ;
			/*(Optional)
			 The Control Unit will be deinitialized and disposed
			 */
			void unitDeinit() ;

		public:
			/*
			 Construct a new CU with an identifier
			 */
			SCUTA(const std::string& _control_unit_id,
									 const std::string& _control_unit_param,
									 const ControlUnitDriverList& _control_unit_drivers);
			
			/*
			 Base destructor
			 */
			~SCUTA();
		};
	}
}

#endif /* defined(__SCUTA__) */
