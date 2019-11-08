//
//  CmdCameraFilterAcquire.cpp
//  PowerSupply
//
//  Created by Claudio Bisegni on 06/11/13.
//  Copyright (c) 2013 infn. All rights reserved.
//

#include "CmdCameraFilterAcquire.h"

#include <boost/format.hpp>

#define CMDCUINFO INFO_LOG(CmdCameraFilterAcquire) << "[" << getDeviceID() << "] "
#define CMDCUDBG DBG_LOG(CmdCameraFilterAcquire) << "[" << getDeviceID() << "] "
#define CMDCUERR ERR_LOG(CmdCameraFilterAcquire) << "[" << getDeviceID() << "] "

namespace own =  driver::powersupply;
namespace c_data = chaos::common::data;
namespace chaos_batch = chaos::common::batch_command;
using namespace chaos::cu::control_manager;

//using  namespace driver::powersupply;
BATCH_COMMAND_OPEN_DESCRIPTION_ALIAS(driver::powersupply::,CmdCameraFilterAcquire,CMD_PS_MODE_ALIAS,
		"Set powersupply mode (on,standby)",
		"34c5dc7e-35ca-11e5-a7ed-971f57b4b945")
BATCH_COMMAND_ADD_INT32_PARAM(CMD_PS_MODE_TYPE, "0:standby, 1:on",chaos::common::batch_command::BatchCommandAndParameterDescriptionkey::BC_PARAMETER_FLAG_MANDATORY)
BATCH_COMMAND_CLOSE_DESCRIPTION()
// return the implemented handler
own::CmdCameraFilterAcquire::~CmdCameraFilterAcquire(){

}
void own::CmdCameraFilterAcquire::setHandler(c_data::CDataWrapper *data) {
	CMDCUINFO << "Executing set handler";

	int err = 0;
	int state = 0;
	std::string state_description;

	AbstractPowerSupplyCommand::setHandler(data);
	AbstractPowerSupplyCommand::acquireHandler();

	setStateVariableSeverity(StateVariableTypeAlarmCU,"stby_invalid_set", chaos::common::alarm::MultiSeverityAlarmLevelClear);


	//requested mode
	if(!data->hasKey(CMD_PS_MODE_TYPE)) {
		CMDCUERR << "Mode type not present";
		setStateVariableSeverity(StateVariableTypeAlarmCU,"stby_invalid_set", chaos::common::alarm::MultiSeverityAlarmLevelWarning);

		BC_FAULT_RUNNING_PROPERTY;
		return;
	}
	state_to_go = data->getInt32Value(CMD_PS_MODE_TYPE);
	if(state_to_go>1) {
		setStateVariableSeverity(StateVariableTypeAlarmCU,"stby_invalid_set", chaos::common::alarm::MultiSeverityAlarmLevelWarning);
		metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelError,boost::str( boost::format("Request mode '%1%' not implemented") % state_to_go) );

		BC_FAULT_RUNNING_PROPERTY;
		return;
	}



	CMDCUINFO << "mode:"<<state_to_go;
	switch (state_to_go) {
	case 0://to standby
		//i need to be in operational to exec
		CMDCUINFO << "Request to go to stanby";

		if((err = powersupply_drv->standby())) {
			metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelError,boost::str( boost::format("Error calling driver for \"standby\" on powersupply err:%1%") % err) );

			//	CMDCUERR<<boost::str( boost::format("Error calling driver for \"standby\" on powersupply err:%1%") % err);
			setStateVariableSeverity(StateVariableTypeAlarmCU,"stby_invalid_set", chaos::common::alarm::MultiSeverityAlarmLevelHigh);

			BC_FAULT_RUNNING_PROPERTY;
			return;
		}
		*i_stby=true;
		break;

	case 1://to operational

		CMDCUINFO << "Request to go to operational";

		if((err = powersupply_drv->poweron())) {
			if(*o_pol == 0 ){
				metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelWarning,boost::str( boost::format("cannot go to operational because of polarity open err:%1% ") % err) );
				setStateVariableSeverity(StateVariableTypeAlarmCU,"stby_invalid_set", chaos::common::alarm::MultiSeverityAlarmLevelWarning);

				BC_FAULT_RUNNING_PROPERTY;
				return;

			}

			metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelError,boost::str( boost::format("Error calling driver for \"operational\" on powersupply err:%1%") % err) );
			setStateVariableSeverity(StateVariableTypeAlarmCU,"stby_invalid_set", chaos::common::alarm::MultiSeverityAlarmLevelHigh);

			BC_FAULT_RUNNING_PROPERTY;
			return;

		}
		*i_stby=false;

		break;
	}

	//set comamnd timeout for this instance
	uint64_t timeo;
	if(*p_setTimeout) {
		CMDCUINFO << "Set time out in "<< *p_setTimeout << "milliseconds";
		//we have a timeout for command so apply it to this instance
		timeo= *p_setTimeout;
	} else {
		CMDCUINFO << "Set time out in milliseconds "<<DEFAULT_COMMAND_TIMEOUT_MS;
		timeo =DEFAULT_COMMAND_TIMEOUT_MS;
		//we have a timeout for command so apply it to this instance
	}
	setFeatures(chaos_batch::features::FeaturesFlagTypes::FF_SET_COMMAND_TIMEOUT, (uint64_t)timeo*1000);

	//send comamnd to driver
	setStateVariableSeverity(StateVariableTypeAlarmCU,"stby_value_not_reached", chaos::common::alarm::MultiSeverityAlarmLevelClear);
	setStateVariableSeverity(StateVariableTypeAlarmCU,"stby_out_of_set",chaos::common::alarm::MultiSeverityAlarmLevelClear);

	getAttributeCache()->setInputDomainAsChanged();
	//run in esclusive mode
	metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelInfo,boost::str( boost::format("performing command mode:%1% timeo %2%2 ms") % state_to_go %timeo) );

	BC_NORMAL_RUNNING_PROPERTY;
}

void own::CmdCameraFilterAcquire::acquireHandler() {
	AbstractPowerSupplyCommand::acquireHandler();

	//force output dataset as changed
	getAttributeCache()->setOutputDomainAsChanged();
}

void own::CmdCameraFilterAcquire::ccHandler() {
	uint64_t elapsed_msec = chaos::common::utility::TimingUtil::getTimeStamp() - getSetTime();


	if(*i_stby==*o_stby){
		CMDCUINFO<<"STATE REACHED stby:"<<*i_stby;
		BC_END_RUNNING_PROPERTY
		return;
	}
	if(*o_alarms) {
		BC_END_RUNNING_PROPERTY
		CMDCUERR << boost::str(boost::format("[metric] Got alarm code %1% in %2% milliseconds") % *o_alarms % elapsed_msec);
	}
}

bool own::CmdCameraFilterAcquire::timeoutHandler() {
	CMDCUINFO << "enter timeoutHandler";
	//move the state machine on fault
	uint64_t elapsed_msec = chaos::common::utility::TimingUtil::getTimeStamp() - getSetTime();
	if(*i_stby==*o_stby){
		CMDCUINFO<<"STATE reached stby:"<<*i_stby;
		BC_END_RUNNING_PROPERTY
		return false;
	} else {
		setStateVariableSeverity(StateVariableTypeAlarmCU,"stby_value_not_reached", chaos::common::alarm::MultiSeverityAlarmLevelWarning);

	}

	BC_END_RUNNING_PROPERTY

	CMDCUINFO << "timeout";
	return false;
}
