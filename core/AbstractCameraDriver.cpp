/*
 *	AbstracCameraDriver.cpp
 *	!CHAOS
 *	Created by Andrea Michelotti
 *  Abstraction of a simple sensor driver
 *    	Copyright 2018 INFN, National Institute of Nuclear Physics
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
#include "AbstractCameraDriver.h"
namespace driver {
namespace sensor {
namespace camera{
#define AbstractCameraDriverLAPP_		LAPP_ << "[AbstractCameraDriver] "
#define AbstractCameraDriverLDBG_		LDBG_ << "[AbstractCameraDriver:"<<__FUNCTION__<<"]"
#define AbstractCameraDriverLERR_		LERR_ << "[AbstractCameraDriver:"<<__PRETTY_FUNCTION__<<"]"

void AbstractCameraDriver::parseInitCommonParams(const chaos::common::data::CDataWrapper& config){
    AbstractCameraDriverLDBG_<<"config:"<<config.getCompliantJSONString();
    if(config.hasKey(TRIGGER_HW_SOURCE_KEY)){
        triggerHWSource=config.getStringValue(TRIGGER_HW_SOURCE_KEY);
    }
    if(config.hasKey(SERIAL_KEY)){
        serial=config.getStringValue(SERIAL_KEY);
        AbstractCameraDriverLDBG_<<"seria:"<<serial;

    }
    if(config.hasKey(TRIGGER_HW_TIMEOUT_KEY)){
        hw_trigger_timeout_us=config.getInt32Value(TRIGGER_HW_TIMEOUT_KEY);
    }
    if(config.hasKey(TRIGGER_SW_TIMEOUT_KEY)){
        sw_trigger_timeout_us=config.getInt32Value(TRIGGER_SW_TIMEOUT_KEY);
    }
     if(config.hasKey(TRIGGER_MODE_KEY)){
        tmode=(TriggerModes)config.getInt32Value(TRIGGER_MODE_KEY);
    }
    if(config.hasKey(GRAB_STRATEGY_KEY)){
        gstrategy=(GrabStrategy)config.getInt32Value("GRAB_STRATEGY");
    }
}

}
}
}
