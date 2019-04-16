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
void AbstractCameraDriver::parseInitCommonParams(const chaos::common::data::CDataWrapper& props){
    if(props.hasKey(TRIGGER_HW_SOURCE_KEY)){
        triggerHWSource=props.getStringValue(TRIGGER_HW_SOURCE_KEY);
    }
    if(props.hasKey(SERIAL_KEY)){
        serial=props.getStringValue(SERIAL_KEY);
    }
    if(props.hasKey(TRIGGER_HW_TIMEOUT_KEY)){
        hw_trigger_timeout_us=props.getInt32Value(TRIGGER_HW_TIMEOUT_KEY);
    }
    if(props.hasKey(TRIGGER_SW_TIMEOUT_KEY)){
        sw_trigger_timeout_us=props.getInt32Value(TRIGGER_SW_TIMEOUT_KEY);
    }
     if(props.hasKey(TRIGGER_MODE_KEY)){
        tmode=props.getInt32Value(TRIGGER_MODE_KEY);
    }
}

}
}
}
#endif /* defined(__ControlUnitTest__DummyDriver__) */
