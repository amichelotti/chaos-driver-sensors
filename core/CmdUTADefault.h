/*
 *	CmdUTADefault.h
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

#ifndef __CmdUTADefault__
#define __CmdUTADefault__

#include "SensorDriverInterface.h"
#include "SCUTA.h"
#include <chaos/cu_toolkit/control_manager/slow_command/SlowCommand.h>

namespace c_data = chaos::common::data;
namespace ccc_slow_command = chaos::cu::control_manager::slow_command;

namespace driver {
	namespace sensor {
		
		DEFINE_BATCH_COMMAND_CLASS(CmdUTADefault,ccc_slow_command::SlowCommand) {
			

		protected:
                        double		*temp[4];
			double		*shutter[4];
			SensorDriverInterface *driver[UTA_DRIVERS];
			// return the implemented handler
			uint8_t implementedHandler();
			
			// Start the command execution
			void setHandler(c_data::CDataWrapper *data);
			
			// Aquire the necessary data for the command
			/*!
			 The acquire handler has the purpose to get all necessary data need the by CC handler.
			 \return the mask for the runnign state
			 */
			void acquireHandler();
		public:
			CmdUTADefault();
			~CmdUTADefault();
		};
		
	}
}


#endif /* defined(__CmdUTADefault__) */
