/*
 *	SCUTA
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

#include "SCUTA.h"
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>

//---comands----
#include "CmdUTADefault.h"
#include "CmdUTAShutter.h"


using namespace chaos;
using namespace chaos::common::data;
using namespace chaos::common::batch_command;

using namespace chaos::cu::control_manager::slow_command;
using namespace chaos::cu::driver_manager::driver;


#define SCCUAPP INFO_LOG(SCUTA)
#define SCCUDBG DBG_LOG(SCUTA)
#define SCCUERR ERR_LOG(SCUTA)

PUBLISHABLE_CONTROL_UNIT_IMPLEMENTATION(::driver::sensor::SCUTA)

/*
 Construct a new CU with an identifier
 */
  ::driver::sensor::SCUTA::SCUTA(const string &_control_unit_id,
                                                                          const string &_control_unit_param,
                                                                          const ControlUnitDriverList &_control_unit_drivers)
    :
//call base constructor
    chaos::cu::control_manager::SCAbstractControlUnit(_control_unit_id,
                                                      _control_unit_param,
                                                      _control_unit_drivers) {
      int cnt;
    for(cnt=0;cnt<UTA_DRIVERS;cnt++){
        driver[cnt]=NULL;
        
    }
}

/*
 Base destructor
 */
::driver::sensor::SCUTA::~SCUTA() {
  int cnt;
  for(cnt=0;cnt<UTA_DRIVERS;cnt++){
    if(driver[cnt]){
      delete driver[cnt];
      driver[cnt]=0;
    }
  }
}


/*
 Return the default configuration
 */
void ::driver::sensor::SCUTA::unitDefineActionAndDataset() throw(chaos::CException) {
  //install all command
  installCommand(BATCH_COMMAND_GET_DESCRIPTION(CmdUTADefault), true);
  installCommand(BATCH_COMMAND_GET_DESCRIPTION(CmdUTAShutter));

  //setup the dataset
  addAttributeToDataSet("I_TEMP",
                        "Input Temperature",
                        DataType::TYPE_DOUBLE,
                        DataType::Output);

  addAttributeToDataSet("O_TEMP",
                        "Output Temperature",
                        DataType::TYPE_DOUBLE,
                        DataType::Output);

  addAttributeToDataSet("I_SHUTTER",
                        "Input Shutter Open Percentage 0=close 100=totally open",
                        DataType::TYPE_DOUBLE,
                        DataType::Output);

  addAttributeToDataSet("O_SHUTTER",
                        "Output Shutter Open Percentage 0=close 100=totally open",
                        DataType::TYPE_DOUBLE,
                        DataType::Output);


}

void ::driver::sensor::SCUTA::unitDefineCustomAttribute() {

}

// Abstract method for the initialization of the control unit
void ::driver::sensor::SCUTA::unitInit() throw(CException) {
  int cnt;
  DPRINT("initializing");
  
  for(cnt=0;cnt<UTA_DRIVERS;cnt++){
    chaos::cu::driver_manager::driver::DriverAccessor * accessor= getAccessoInstanceByIndex(cnt);
    if (accessor == NULL) {
      throw chaos::CException(-1, "Cannot retrieve driver", __FUNCTION__);
    }
    
    driver[cnt] = new ::driver::sensor::SensorDriverInterface(accessor);
    if (driver[cnt] == NULL) {
      throw chaos::CException(-2, "Cannot allocate driver resources", __FUNCTION__);
    }
  }
  

}

// Abstract method for the start of the control unit
void ::driver::sensor::SCUTA::unitStart() throw(CException) {
      double *shutter;
      double val=5;
  DPRINT("initializing shutters to 50%");

shutter= getAttributeCache()->getRWPtr<double>(DOMAIN_OUTPUT, "I_SHUTTER");
  *shutter = 50;
  shutter= getAttributeCache()->getRWPtr<double>(DOMAIN_OUTPUT, "O_SHUTTER");
  *shutter = 50;
  
  driver[UTA_SHUTTER_DRIVER]->writeChannel(&val,0,sizeof(double));
  driver[UTA_SHUTTER_DRIVER]->writeChannel(&val,1,sizeof(double));
  getAttributeCache()->setOutputDomainAsChanged();
}

// Abstract method for the stop of the control unit
void ::driver::sensor::SCUTA::unitStop() throw(CException) {

}

// Abstract method for the deinit of the control unit
void ::driver::sensor::SCUTA::unitDeinit() throw(CException) {

}

