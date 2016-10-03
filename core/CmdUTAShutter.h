/*
 *	CmdUTAShutter.h
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

#ifndef __PowerSupply__CmdSetCurrent__
#define __PowerSupply__CmdSetCurrent__

#include "CmdUTADefault.h"

#include <bitset>

namespace c_data = chaos::common::data;
namespace ccc_slow_command = chaos::cu::control_manager::slow_command;

namespace driver {
	namespace sensor {
		
		//! Command for change the mode of the powersupply
		DEFINE_BATCH_COMMAND_CLASS(CmdUTAShutter,CmdUTADefault) {
	
		protected:
			//implemented handler
			uint8_t implementedHandler();
			
			// Set handler
			void setHandler(c_data::CDataWrapper *data);

			//custom acquire method
			void acquireHandler();

			//Correlation and commit phase
			//void ccHandler();
			
			//manage the timeout
			//bool timeoutHandler();
		};
	}
}


#endif /* defined(__PowerSupply__CmdSetCurrent__) */
