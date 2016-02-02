/*
 *	CmdUTADefault.cpp
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

#include <string.h>
#include "CmdUTADefault.h"
#include <common/debug/core/debug.h>

#define LOG_HEAD_CmdUTADefault LOG_TAIL(CmdUTADefault)

#define CMDCU_ INFO_LOG(CmdUTADefault)
#define CMDCUDBG_ DBG_LOG(CmdUTADefault)
#define CMDCUERR_ ERR_LOG(CmdUTADefault)

using namespace chaos::common::data;
using namespace chaos::common::batch_command;
using namespace chaos::cu::control_manager::slow_command;

using namespace driver::sensor;
BATCH_COMMAND_OPEN_DESCRIPTION(driver::sensor::,CmdUTADefault,
			       "Default method",
			       "4403149a-35df-11e5-b1f7-7f8214ad6212")
BATCH_COMMAND_CLOSE_DESCRIPTION()
CmdUTADefault::CmdUTADefault() {
    for(int cnt=0;cnt<UTA_DRIVERS;cnt++){
        driver[cnt]=NULL;
        
        
    }
	
}

CmdUTADefault::~CmdUTADefault() {
    int cnt;
    for(cnt=0;cnt<UTA_DRIVERS;cnt++){
        if(driver[cnt]){
            DPRINT("deleting driver %d",cnt);
            delete driver[cnt];
        }
        driver[cnt]=0;
    }
    
	
}

    // return the implemented handler
uint8_t CmdUTADefault::implementedHandler() {
        //add to default hadnler the acquisition one
	return  chaos_batch::HandlerType::HT_Set | chaos_batch::HandlerType::HT_Acquisition;
}

    // Start the command execution
void CmdUTADefault::setHandler(c_data::CDataWrapper *data) {
	

	//set the default scheduling to one seconds
	setFeatures(features::FeaturesFlagTypes::FF_SET_SCHEDULER_DELAY, (uint64_t)1000000);

	//get channel pointer
	temp[0] = getAttributeCache()->getRWPtr<double>(DOMAIN_OUTPUT, "I_TEMP");
	temp[1] = getAttributeCache()->getRWPtr<double>(DOMAIN_OUTPUT, "O_TEMP");
        shutter[0] = getAttributeCache()->getRWPtr<double>(DOMAIN_OUTPUT, "I_SHUTTER");
	shutter[1] = getAttributeCache()->getRWPtr<double>(DOMAIN_OUTPUT, "O_SHUTTER");
    for(int cnt=0;cnt<UTA_DRIVERS;cnt++){
        chaos::cu::driver_manager::driver::DriverAccessor *accessor =driverAccessorsErogator->getAccessoInstanceByIndex(cnt); 
        
        if (accessor== NULL) {
            throw chaos::CException(-1, "Cannot retrieve driver", __FUNCTION__);
        }
    
        driver[cnt]= new driver::sensor::SensorDriverInterface(accessor);
    
        if (driver[cnt]== NULL) {
          throw chaos::CException(-2, "Cannot retrieve driver", __FUNCTION__);
        }
    }
    
	BC_NORMAL_RUNNIG_PROPERTY
}

    // Aquire the necessary data for the command
/*!
 The acquire handler has the purpose to get all necessary data need the by CC handler.
 \return the mask for the runnign state
 */
void CmdUTADefault::acquireHandler() {
	string desc;
	int err = 0;
	int stato = 0;
	float tmp_float = 0.0F;
	int tmp_uint32 = 0;
	uint64_t tmp_uint64 = 0;
	CMDCU_ << "Acquiring data";
        
        driver[UTA_TERMOMETER_DRIVER]->readChannel(temp[0],0,sizeof(double));
        driver[UTA_TERMOMETER_DRIVER]->readChannel(temp[1],1,sizeof(double));
        CMDCUDBG_<<"TEMP0:"<<*temp[0]<<" TEMP1:"<<*temp[1];
	//force output dataset as changed
	getAttributeCache()->setOutputDomainAsChanged();
}
