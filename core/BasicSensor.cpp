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
using namespace ::driver::sensor;
using namespace chaos::cu::control_manager;
using namespace chaos::cu::driver_manager::driver;

PUBLISHABLE_CONTROL_UNIT_IMPLEMENTATION(BasicSensor)

#define BasicSensorLAPP_		LAPP_ << "[BasicSensor] "
#define BasicSensorLDBG_		LDBG_ << "[BasicSensor] " << __PRETTY_FUNCTION__ << " "
#define BasicSensorLERR_		LERR_ << "[BasicSensor] " << __PRETTY_FUNCTION__ << "("<<__LINE__<<") "

/*
 Construct
 */
BasicSensor::BasicSensor(const string& _control_unit_id, const string& _control_unit_param, const ControlUnitDriverList& _control_unit_drivers):last_update(0),sensor_timeout(120*1000), 
RTAbstractControlUnit(_control_unit_id, _control_unit_param, _control_unit_drivers){
   BasicSensorLAPP_ << "Created";
   driver=NULL;
   try{
         chaos::common::data::CDataWrapper parms;
        parms.setSerializedJsonData(_control_unit_param.c_str());
        if(parms.hasKey("sensor_timeout")){
            sensor_timeout = parms.getInt32Value("sensor_timeout");
        }

   } catch(...){

   }
}

/*
 Destructor
 */
BasicSensor::~BasicSensor() {
    if(driver){
        driver->deinitIO();
        delete driver;
        driver = NULL;
    }
   /* if(driver_dataset){
        free(driver_dataset);
        driver_dataset=0;
    }*/

}

//!Return the definition of the control unit
/*!
The api that can be called withi this method are listed into
"Control Unit Definition Public API" module into html documentation
(chaosframework/Documentation/html/group___control___unit___definition___api.html)
*/
void BasicSensor::unitDefineActionAndDataset()  {
    int ret;
    BasicSensorLAPP_ << "UnitDefine";
    driver=new chaos::cu::driver_manager::driver::BasicIODriverInterface(getAccessoInstanceByIndex(0));

  assert(driver);
  /*driver_dataset_size=driver->getDatasetSize();
  driver_dataset=0;
  if(driver_dataset_size>0){
    driver_dataset = (ddDataSet_t *)malloc( driver_dataset_size);
    assert(driver_dataset);
  }
  addStateVariable(StateVariableTypeAlarmCU,"timeout_sensor_readout","Sensor timeout");
    ret=driver->getDataset(driver_dataset,driver_dataset_size);
    
    for(int cnt=0;cnt<ret/sizeof(ddDataSet_t);cnt++){
        BasicSensorLDBG_<<"adding attribute:"<<driver_dataset[cnt].name<<","<<driver_dataset[cnt].desc<<","<<driver_dataset[cnt].type<<","<<driver_dataset[cnt].dir<<","<<driver_dataset[cnt].maxsize;
        if(driver_dataset[cnt].dir==chaos::DataType::Input){
            input_size.push_back(driver_dataset[cnt].maxsize);
        } else if(driver_dataset[cnt].dir==chaos::DataType::Output){
            output_size.push_back(driver_dataset[cnt].maxsize);

        }
        addAttributeToDataSet(driver_dataset[cnt].name,
                              driver_dataset[cnt].desc,
                              driver_dataset[cnt].type,
                              driver_dataset[cnt].dir,
                              driver_dataset[cnt].maxsize
                              );

    }
*/
      addPublicDriverPropertyToDataset();

  }


//!Define custom control unit attribute
void BasicSensor::unitDefineCustomAttribute() {

}

//!Initialize the Custom Control Unit
void BasicSensor::unitInit()  {
getAttributeCache()->resetChangedInputIndex();

}

//!Execute the work, this is called with a determinated delay, it must be as fast as possible
void BasicSensor::unitStart()  {
    //setDefaultScheduleDelay(1000000);
    //setStateVariableSeverity(StateVariableTypeAlarmCU,"timeout_sensor_readout", chaos::common::alarm::MultiSeverityAlarmLevelClear);

}

//!Execute the Control Unit work
void BasicSensor::unitRun()  {
  //get the output attribute pointer form the internal cache
    std::vector<int>::iterator i;
    int cnt,ret,changed=0;
    //BasicSensorLAPP_<<"UnitRun";
    updateDatasetFromDriverProperty();
/*
    for(i=output_size.begin(),cnt=0;i!=output_size.end();i++,cnt++){
        char buffer[*i];
        if((ret=driver->read(buffer,cnt,*i))){
	        changed++;
          //  BasicSensorLDBG_<<"Reading output channel "<<cnt<<", size :"<<*i <<" ret:"<<ret;
            getAttributeCache()->setOutputAttributeValue(cnt, (void*)buffer, *i);
        }
    }
    uint64_t difftime=(chaos::common::utility::TimingUtil::getLocalTimeStamp() - last_update);
  //! set output dataset as changed
    if(changed){
      setStateVariableSeverity(StateVariableTypeAlarmCU,"timeout_sensor_readout", chaos::common::alarm::MultiSeverityAlarmLevelClear);
      last_update=chaos::common::utility::TimingUtil::getLocalTimeStamp();
      getAttributeCache()->setOutputDomainAsChanged();
    } else if((difftime> sensor_timeout)&&(difftime<2*sensor_timeout)){
        setStateVariableSeverity(StateVariableTypeAlarmCU,"timeout_sensor_readout", chaos::common::alarm::MultiSeverityAlarmLevelWarning);

    } else if(difftime> 2*sensor_timeout){
        setStateVariableSeverity(StateVariableTypeAlarmCU,"timeout_sensor_readout", chaos::common::alarm::MultiSeverityAlarmLevelHigh);

    }
    */
}

//!Execute the Control Unit work
void BasicSensor::unitStop()  {

}

//!Deinit the Control Unit
void BasicSensor::unitDeinit()  {

}

//! attribute changed handler
bool BasicSensor::unitInputAttributePreChangeHandler(chaos::common::data::CDWUniquePtr& data){
    std::vector<VariableIndexType> changed;
    std::vector<VariableIndexType>::iterator j;
    getAttributeCache()->getChangedInputAttributeIndex(changed);
    //BasicSensorLAPP_<<"UnitRun";
    
    for(j=changed.begin();j!=changed.end();j++){
        const char** buffer;
        
        getAttributeCache()->getReadonlyCachedAttributeValue<char>(DOMAIN_INPUT, *j, &buffer);
        if(driver->write(buffer,*j,input_size[*j])){
            BasicSensorLDBG_<<"writing output channel "<<*j<<", size :"<<input_size[*j];
        }
    }
    getAttributeCache()->resetChangedInputIndex();

return true;
}

/*
CDataWrapper *BasicSensor::my_custom_action(CDataWrapper *actionParam, bool& detachParam) {
	CDataWrapper *result =  new CDataWrapper();
	return result;
}
*/
