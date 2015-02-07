/*
 *	DummyDriver.h
 *	!CHOAS
 *	Created by Bisegni Claudio.
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
#include "BasicSensorDriver.h"

#include <string>

#include <chaos/cu_toolkit/driver_manager/driver/AbstractDriverPlugin.h>

#include <boost/lexical_cast.hpp>

namespace cu_driver = chaos::cu::driver_manager::driver;

#define BasicSensorDriverLAPP_		LAPP_ << "[BasicSensorDriver] "
#define BasicSensorDriverLDBG_		LDBG_ << "[BasicSensorDriver] "
#define BasicSensorDriverLERR_		LERR_ << "[BasicSensorDriver] "


//GET_PLUGIN_CLASS_DEFINITION
//we need to define the driver with alias version and a class that implement it
OPEN_CU_DRIVER_PLUGIN_CLASS_DEFINITION(BasicSensorDriver, 1.0.0, BasicSensorDriver)
//we need to describe the driver with a parameter string
REGISTER_CU_DRIVER_PLUGIN_CLASS_INIT_ATTRIBUTE(BasicSensorDriver, unsigned 32 bit)
CLOSE_CU_DRIVER_PLUGIN_CLASS_DEFINITION

//default constructor definition
DEFAULT_CU_DRIVER_PLUGIN_CONSTRUCTOR(BasicSensorDriver) {

}

//default descrutcor
BasicSensorDriver::~BasicSensorDriver() {

}

void BasicSensorDriver::driverInit(const char *initParameter) throw(chaos::CException) {
	BasicSensorDriverLAPP_ << "Init driver";
	try{
		i32_out_1_value = boost::lexical_cast<int32_t>(initParameter);
	} catch(...) {
		throw chaos::CException(-1, "Error on seed value", __PRETTY_FUNCTION__);
	}
	BasicSensorDriverLAPP_ << "inizialised driver with seed: " << i32_out_1_value;
}

void BasicSensorDriver::driverDeinit() throw(chaos::CException) {
	BasicSensorDriverLAPP_ << "Deinit driver";

}

//! Execute a command
cu_driver::MsgManagmentResultType::MsgManagmentResult BasicSensorDriver::execOpcode(cu_driver::DrvMsgPtr cmd) {
	cu_driver::MsgManagmentResultType::MsgManagmentResult result = cu_driver::MsgManagmentResultType::MMR_EXECUTED;
	switch(cmd->opcode) {
		case BasicSensorDriverOpcode_SET_CH_1:
			if(!cmd->inputData  || (sizeof(int32_t) != cmd->inputDataLength)) break;
			//we can get the value
			i32_out_1_value = *static_cast<int32_t*>(cmd->inputData);
		break;

		case BasicSensorDriverOpcode_GET_CH_1:
			*static_cast<int32_t*>(cmd->resultData) = i32_out_1_value;
		break;
	}
	return result;
}
