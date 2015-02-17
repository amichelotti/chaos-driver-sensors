/*
 *	AbstractSensorDriver.cpp
 *	!CHAOS
 *	Created by Andrea Michelotti
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
#include "AbstractSensorDriver.h"
#include <stdlib.h>
#include <string>

#include <chaos/cu_toolkit/driver_manager/driver/AbstractDriverPlugin.h>

#include <boost/lexical_cast.hpp>

namespace cu_driver = chaos::cu::driver_manager::driver;

#define AbstractSensorDriverLAPP_		LAPP_ << "[AbstractSensorDriver] "
#define AbstractSensorDriverLDBG_		LDBG_ << "[AbstractSensorDriver] "
#define AbstractSensorDriverLERR_		LERR_ << "[AbstractSensorDriver] "


//GET_PLUGIN_CLASS_DEFINITION
//we need to define the driver with alias version and a class that implement it
//default constructor definition
DEFAULT_CU_DRIVER_PLUGIN_CONSTRUCTOR(AbstractSensorDriver) {
}


//default descrutcor
AbstractSensorDriver::~AbstractSensorDriver() {

}

void AbstractSensorDriver::driverInit(const char *initParameter) throw(chaos::CException) {
	AbstractSensorDriverLAPP_ << "Init driver";
    sensorInit((void*)initParameter,(int)strlen(initParameter));
	
}

void AbstractSensorDriver::driverDeinit() throw(chaos::CException) {
	AbstractSensorDriverLAPP_ << "Deinit driver";
    sensorDeinit();

}

//! Execute a command
cu_driver::MsgManagmentResultType::MsgManagmentResult AbstractSensorDriver::execOpcode(cu_driver::DrvMsgPtr cmd) {
	cu_driver::MsgManagmentResultType::MsgManagmentResult result = cu_driver::MsgManagmentResultType::MMR_EXECUTED;
	switch(cmd->opcode) {
		case AbstractSensorDriverOpcode_READ_CHANNEL:{
            int ch=cmd->parm[0];
            int sizeb=cmd->parm[1];
            cmd->resultDataLength= sizeb;
            AbstractSensorDriverLDBG_<<"Read channel:"<<ch<<" size:"<<sizeb;
            cmd->ret=readChannel(cmd->resultData,ch,sizeb);
            break;
        }

		case AbstractSensorDriverOpcode_WRITE_CHANNEL:{
            int ch=cmd->parm[0];
            int sizeb=cmd->parm[1];

            cmd->inputDataLength= sizeb;
            AbstractSensorDriverLDBG_<<"Write channel:"<<ch<<" size:"<<sizeb;
            cmd->ret=writeChannel(cmd->inputData,ch,sizeb);
            break;
        }
        case AbstractSensorDriverOpcode_INIT:{
            AbstractSensorDriverLDBG_<<"Init";
            int sizeb=cmd->parm[0];
            cmd->inputDataLength=sizeb;
            cmd->ret=sensorInit(cmd->inputData,sizeb);
            break;
        }
        case AbstractSensorDriverOpcode_DEINIT:{
            AbstractSensorDriverLDBG_<<"DeInit";

            cmd->ret=sensorDeinit();
            break;
        }
        case AbstractSensorDriverOpcode_GET_DATASET:{
            ddDataSet_t*dataset=(ddDataSet_t*)cmd->resultData;

            cmd->ret = getDataset(dataset,cmd->resultDataLength);
        }
        break;
        case AbstractSensorDriverOpcode_GET_DATASETSIZE:{
            cmd->ret = getDatasetSize();

        }
        break;
	}
	return result;
}

int AbstractSensorDriver::getDatasetSize(){
  return datasetSize;
}
int AbstractSensorDriver::getDataset(ddDataSet_t*data,int sizeb){
    int ret=sizeb<datasetSize?sizeb:getDatasetSize();
    memcpy(data,dataset,ret);
    return ret;

}
void AbstractSensorDriver::setDataSet(ddDataSet_t*data,int sizeb){
    dataset = data;
    datasetSize=sizeb;
}
