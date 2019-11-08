/*
 *	CmdCameraFilterDefault.cpp
 *	!CHOAS
 *	Created by Claudio Bisegni.
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
#include "CmdCameraFilterDefault.h"

#define LOG_HEAD_CmdCameraFilterDefault LOG_TAIL(CmdCameraFilterDefault)

#define CMDCU_ INFO_LOG(CmdCameraFilterDefault) << "[" << getDeviceID() << "] "
#define CMDCUDBG_ DBG_LOG(CmdCameraFilterDefault) << "[" << getDeviceID() << "] "
#define CMDCUERR_ ERR_LOG(CmdCameraFilterDefault) << "[" << getDeviceID() << "] "

using namespace chaos::common::data;
using namespace chaos::common::batch_command;
using namespace chaos::cu::control_manager::slow_command;
using namespace chaos::cu::control_manager;
using namespace driver::sensor::camera;

BATCH_COMMAND_OPEN_DESCRIPTION(driver::sensor::camera::,CmdCameraFilterDefault,
		"Default Acquire continuos",
		"4403148a-35df-11e5-b1f7-7f8214ad6212")
BATCH_COMMAND_CLOSE_DESCRIPTION()
CmdCameraFilterDefault::CmdCameraFilterDefault() {
}

CmdCameraFilterDefault::~CmdCameraFilterDefault() {

}

// return the implemented handler

// Start the command execution
void CmdCameraFilterDefault::setHandler(c_data::CDataWrapper *data) {

	AbstractPowerSupplyCommand::setHandler(data);
	metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelInfo,"Entering in Default");

	BC_NORMAL_RUNNING_PROPERTY
}

// Aquire the necessary data for the command
/*!
 The acquire handler has the purpose to get all necessary data need the by CC handler.
 \return the mask for the runnign state
 */
void CmdCameraFilterDefault::acquireHandler() {
	AbstractPowerSupplyCommand::acquireHandler();
	//force output dataset as changed
	cv::Mat image(*sizey, *sizex, framebuf_encoding, a.buf);

}
#define SETDEVALARM(x,y,desc) \
		if((x)&y){\
			setStateVariableSeverity(StateVariableTypeAlarmDEV,desc, chaos::common::alarm::MultiSeverityAlarmLevelHigh);\
		}

void CmdCameraFilterDefault::ccHandler() {
	/////  CHECKS during operational mode
	if(*i_stby!=*o_stby){
		setStateVariableSeverity(StateVariableTypeAlarmCU,"stby_out_of_set",chaos::common::alarm::MultiSeverityAlarmLevelWarning);
	} else {
		setStateVariableSeverity(StateVariableTypeAlarmCU,"stby_out_of_set",chaos::common::alarm::MultiSeverityAlarmLevelClear);
		setStateVariableSeverity(StateVariableTypeAlarmCU,"stby_value_not_reached",chaos::common::alarm::MultiSeverityAlarmLevelClear);
	}
	if(*o_alarms){
		SETDEVALARM(*o_alarms,POWER_SUPPLY_EVENT_DOOR_OPEN,"door_open");
		SETDEVALARM(*o_alarms,POWER_SUPPLY_EVENT_OVER_TEMP,"over_temp");
		SETDEVALARM(*o_alarms,POWER_SUPPLY_FUSE_FAULT,"fuse_fault");
		SETDEVALARM(*o_alarms,POWER_SUPPLY_EARTH_FAULT,"earth_fault");
		SETDEVALARM(*o_alarms,POWER_SUPPLY_OVER_VOLTAGE,"over_voltage");
		SETDEVALARM(*o_alarms,POWER_SUPPLY_OVER_CURRENT,"over_current");
		SETDEVALARM(*o_alarms,POWER_SUPPLY_COMMUNICATION_FAILURE,"communication_failure");
		SETDEVALARM(*o_alarms,POWER_SUPPLY_MAINUNIT_FAIL,"mainunit_failure");
		SETDEVALARM(*o_alarms,POWER_SUPPLY_EXTERNAL_INTERLOCK,"external_interlock");
		SETDEVALARM(*o_alarms,POWER_SUPPLY_SETPOINT_CARD_FAULT,"card_fault");
		SETDEVALARM(*o_alarms,POWER_SUPPLY_CUBICLE_OVT,"cubicle_over_temp");
		SETDEVALARM(*o_alarms,POWER_SUPPLY_DCCT_OVT,"dcct_fault");
		SETDEVALARM(*o_alarms,POWER_SUPPLY_ACTIVE_FILTER_FUSE,"active_filter_fuse");

		SETDEVALARM(*o_alarms,POWER_SUPPLY_ACTIVE_FILTER_OVT,"active_filter_overtemp");



		setStateVariableSeverity(StateVariableTypeAlarmDEV,"interlock", chaos::common::alarm::MultiSeverityAlarmLevelHigh);
		getAttributeCache()->setOutputDomainAsChanged();

		return;
	} else {
		setStateVariableSeverity(StateVariableTypeAlarmDEV,"interlock", chaos::common::alarm::MultiSeverityAlarmLevelClear);

	}
	setStateVariableSeverity(StateVariableTypeAlarmDEV, chaos::common::alarm::MultiSeverityAlarmLevelClear);


	if(*o_stby == 0){
		if(*p_warningThreshold>0){
			//enable out of set warning

			double err=fabs(*o_current-*i_current);
			if(err>*p_warningThreshold){
				if(start_out_of_set_time==0){
					start_out_of_set_time= chaos::common::utility::TimingUtil::getTimeStamp() ;
				}
				uint64_t tdiff=chaos::common::utility::TimingUtil::getTimeStamp() -start_out_of_set_time;
				if(tdiff>*p_warningThresholdTimeout){
					setStateVariableSeverity(StateVariableTypeAlarmCU,"current_out_of_set", chaos::common::alarm::MultiSeverityAlarmLevelWarning);
					CMDCUDBG_<<"current out of set detected diff:"<< err << " after "<<tdiff<< " ms";

				}

			} else {
				start_out_of_set_time=0;
				setStateVariableSeverity(StateVariableTypeAlarmCU,"current_out_of_set", chaos::common::alarm::MultiSeverityAlarmLevelClear);
				setStateVariableSeverity(StateVariableTypeAlarmCU,"current_value_not_reached", chaos::common::alarm::MultiSeverityAlarmLevelClear);

			}
		}
	}
	if((powersupply_drv->getFeatures()& common::powersupply::POWER_SUPPLY_FEAT_BIPOLAR)&&(*i_pol!=*o_pol)){
		setStateVariableSeverity(StateVariableTypeAlarmCU,"polarity_out_of_set", chaos::common::alarm::MultiSeverityAlarmLevelWarning);

	} else {
		setStateVariableSeverity(StateVariableTypeAlarmCU,"polarity_out_of_set", chaos::common::alarm::MultiSeverityAlarmLevelClear);
		setStateVariableSeverity(StateVariableTypeAlarmCU,"polarity_value_not_reached", chaos::common::alarm::MultiSeverityAlarmLevelClear);
	}
	getAttributeCache()->setOutputDomainAsChanged();
}
