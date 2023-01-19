/*
 *	BasicSensor.h
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
#ifndef ChaosRTControlUnit_BasicSensor_h
#define ChaosRTControlUnit_BasicSensor_h

#include <chaos/cu_toolkit/control_manager/RTAbstractControlUnit.h>
#include <driver/sensors/core/SensorDriverInterface.h>
#include <chaos/cu_toolkit/driver_manager/driver/BasicIODriverInterface.h>
#define MAX_DATASET_SIZE 256
namespace driver{
    
    namespace sensor{
            
class BasicSensor : public chaos::cu::control_manager::RTAbstractControlUnit {
	PUBLISHABLE_CONTROL_UNIT_INTERFACE(BasicSensor);
    
    chaos::cu::driver_manager::driver::BasicIODriverInterface*driver;
    //ddDataSet_t*driver_dataset;
    //int driver_dataset_size;
    std::vector<int> output_size;
    std::vector<int> input_size;
    uint64_t last_update;
    uint32_t sensor_timeout;
public:
    /*!
     Construct a new CU with full constructor
     */
    BasicSensor(const std::string& _control_unit_id, const std::string& _control_unit_param, const ControlUnitDriverList& _control_unit_drivers);
    /*!
     Destructor a new CU
     */
    ~BasicSensor();

protected:
		/*!
		Define the Control Unit Dataset and Actions
		*/
		void unitDefineActionAndDataset();
		/*!(Optional)
		Define the Control Unit custom attribute
		*/
		void unitDefineCustomAttribute();
    /*!(Optional)
     Initialize the Control Unit and all driver, with received param from MetadataServer
     */
		void unitInit() ;
    /*!(Optional)
     Execute the work, this is called with a determinated delay
     */
    void unitStart() ;
    /*!
     Execute the work, this is called with a determinated delay, it must be as fast as possible
     */
    void unitRun() ;

    /*!(Optional)
     The Control Unit will be stopped
     */
    void unitStop() ;

    /*!(Optional)
     The Control Unit will be deinitialized and disposed
     */
    void unitDeinit() ;

		//! Handler called on the update of one or more input attribute
		/*!(Optional)
		After an input attribute has been chacnged this handler
		is called
		*/
bool unitInputAttributePreChangeHandler(chaos::common::data::CDWUniquePtr& data);
};
    }}
#endif
