/*
 *	CmdUTAShutter.cpp
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


#include "CmdUTAShutter.h"

#include <cmath>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <sstream>

namespace c_data = chaos::common::data;
namespace chaos_batch = chaos::common::batch_command;
using namespace  ::driver::sensor;
BATCH_COMMAND_OPEN_DESCRIPTION_ALIAS(,CmdUTAShutter,"shutter",
                                                          "Set the given shutter to a percentage of open",
                                                          "72882f3e-35da-11e5-985f-334fcd6dff23")
BATCH_COMMAND_ADD_INT32_PARAM("id", "index of the shutter (0 is INPUT, 1 is OUTPUT)",chaos::common::batch_command::BatchCommandAndParameterDescriptionkey::BC_PARAMETER_FLAG_MANDATORY)
BATCH_COMMAND_ADD_DOUBLE_PARAM("set", "percentage of open [0=close,100=totally open]",chaos::common::batch_command::BatchCommandAndParameterDescriptionkey::BC_PARAMETER_FLAG_MANDATORY)
BATCH_COMMAND_CLOSE_DESCRIPTION()

// return the implemented handler
uint8_t ::driver::sensor::CmdUTAShutter::implementedHandler(){
    return chaos_batch::HandlerType::HT_Set | chaos_batch::HandlerType::HT_Acquisition;
}

void ::driver::sensor::CmdUTAShutter::setHandler(c_data::CDataWrapper *data) {
    chaos::common::data::RangeValueInfo current_sp_attr_info;
    chaos::common::data::RangeValueInfo attributeInfo;
	CmdUTADefault::setHandler(data);

	int err = 0;
	int id;
        double val;
	
        if(!data ||
	   !data->hasKey("id")) {
            DERR("no id is given");
            	BC_END_RUNNIG_PROPERTY
		return;
        }
        
        if(!data ||
	   !data->hasKey("set")) {
            DERR("no set is given");
            	BC_END_RUNNIG_PROPERTY
		return;
        }
        
        id=data->getInt32Value("id");
        val=data->getDoubleValue("set");
	if(id<0 || id>=3){
            DERR("ID \"%d\" out of range",id);
            BC_END_RUNNIG_PROPERTY
	  return;
        }
        if(val<0 || val>100){
            DERR("shutter value \"%f\" out of range",val);
            BC_END_RUNNIG_PROPERTY;
	  return;
        }
        *shutter[id] =val;
        val=(val/100)*10.0; // value between 0 and 10
        driver[UTA_SHUTTER_DRIVER]->writeChannel(&val,id,sizeof(double));
         
	BC_EXEC_RUNNIG_PROPERTY;

}

void ::driver::sensor::CmdUTAShutter::acquireHandler() {
	//force output dataset as changed
	getAttributeCache()->setOutputDomainAsChanged();
        BC_END_RUNNIG_PROPERTY;
        
}
