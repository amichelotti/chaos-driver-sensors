/*
 *	BasicSensor.cpp
 *	!CHAOS
 *	Created by Andrea Michelotti
 *
 *    	Copyright 2012 INFN, National Institute of Nuclear Physics
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

#include "BasicSensor.h"
#include "AbstractSensorDriver.h"

using namespace chaos;
using namespace chaos::common::data::cache;
using namespace chaos::cu::driver_manager::driver;

PUBLISHABLE_CONTROL_UNIT_IMPLEMENTATION(BasicSensor)

#define BasicSensorLAPP_		LAPP_ << "[BasicSensor] "
#define BasicSensorLDBG_		LDBG_ << "[BasicSensor] " << __PRETTY_FUNCTION__ << " "
#define BasicSensorLERR_		LERR_ << "[BasicSensor] " << __PRETTY_FUNCTION__ << "("<<__LINE__<<") "

/*
 Construct
 */
BasicSensor::BasicSensor(const string& _control_unit_id, const string& _control_unit_param, const ControlUnitDriverList& _control_unit_drivers):
RTAbstractControlUnit(_control_unit_id, _control_unit_param, _control_unit_drivers) {

    driver=new SensorDriverInterface(getAccessoInstanceByIndex(0));
    assert(driver);
}

/*
 Destructor
 */
BasicSensor::~BasicSensor() {
    if(driver){
        delete driver;
        driver = NULL;
    }

}

//!Return the definition of the control unit
/*!
The api that can be called withi this method are listed into
"Control Unit Definition Public API" module into html documentation
(chaosframework/Documentation/html/group___control___unit___definition___api.html)
*/
void BasicSensor::unitDefineActionAndDataset() throw(chaos::CException) {
    int ret;
    ddDataSet_t dd[MAX_DATASET_SIZE];
    ret=driver->getDataset(dd,MAX_DATASET_SIZE);
    for(int cnt=0;cnt<ret;cnt++){
        BasicSensorLDBG_<<"adding attribute:"<<dd[cnt].name<<","<<dd[cnt].desc<<","<<dd[cnt].type<<","<<dd[cnt].dir<<","<<dd[cnt].maxsize;
        if(dd[cnt].dir==chaos::DataType::Input){
            input_size.push_back(dd[cnt].maxsize);
        } else if(dd[cnt].dir==chaos::DataType::Output){
            output_size.push_back(dd[cnt].maxsize);

        }
        addAttributeToDataSet(dd[cnt].name,
                              dd[cnt].desc,
                              dd[cnt].type,
                              dd[cnt].dir,
                              dd[cnt].maxsize
                              );

    }
  }


//!Define custom control unit attribute
void BasicSensor::unitDefineCustomAttribute() {

}

//!Initialize the Custom Control Unit
void BasicSensor::unitInit() throw(chaos::CException) {


}

//!Execute the work, this is called with a determinated delay, it must be as fast as possible
void BasicSensor::unitStart() throw(chaos::CException) {

}

//!Execute the Control Unit work
void BasicSensor::unitRun() throw(chaos::CException) {
  //get the output attribute pointer form the internal cache
    std::vector<int>::iterator i;
    int cnt;
    for(i=output_size.begin(),cnt=0;i!=output_size.end();i++,cnt++){
        char buffer[*i];
        if(driver->readChannel(buffer,cnt,*i)){
            BasicSensorLDBG_<<"reading output channel "<<cnt<<", size :"<<*i;
            getAttributeCache()->setOutputAttributeValue(cnt, (void*)buffer, *i);
        }
    }
  
  //! set output dataset as changed
  getAttributeCache()->setOutputDomainAsChanged();
}

//!Execute the Control Unit work
void BasicSensor::unitStop() throw(chaos::CException) {

}

//!Deinit the Control Unit
void BasicSensor::unitDeinit() throw(chaos::CException) {

}

//! pre imput attribute change
void BasicSensor::unitInputAttributePreChangeHandler() throw(chaos::CException) {

}

//! attribute changed handler
void BasicSensor::unitInputAttributeChangedHandler() throw(chaos::CException) {
}

/*
CDataWrapper *BasicSensor::my_custom_action(CDataWrapper *actionParam, bool& detachParam) {
	CDataWrapper *result =  new CDataWrapper();
	return result;
}
*/
