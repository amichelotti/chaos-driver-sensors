/*
 *	VTemperatureDriver.h
 *  Software emulated temperature driver
 *	!CHOAS
 *	Created by Andrea Michelotti
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
#ifndef VTEMPERATUREDRIVER_h
#define VTEMPERATUREDRIVER_h

#include <chaos/cu_toolkit/driver_manager/driver/AbstractDriverPlugin.h>
#include <chaos/cu_toolkit/driver_manager/driver/BasicIODriver.h>
#include <chaos/common/data/DatasetDB.h>
#include <stdint.h>
namespace cu_driver = chaos::cu::driver_manager::driver;

/*
 driver definition
 */
DEFINE_CU_DRIVER_DEFINITION_PROTOTYPE(VTemperatureDriver)

namespace driver {
    namespace sensor {
        namespace model {
    

class VTemperatureDriver:public chaos::cu::driver_manager::driver::BasicIODriver {

	int counter;
    double freq,temp;
    int32_t points;
    int32_t sinpoint;
    double setPoint,err;
    int tempType;
public:
	VTemperatureDriver();
	~VTemperatureDriver();
    //! Execute a command
        /**
         \brief Read a channel from the physical sensor
         \param buffer[out] destination buffer
         \param addr[in] channel address or identification
         \param bcout[in] buffer count
         \return the number of succesful read items, negative error

         */
    int read(void *buffer,int addr,int bcount);
    /**
     \brief Write a channel from the physical sensor
     \param buffer[out] destination buffer
     \param addr[in] channel address or identification
     \param bcout[in] buffer count
     \return the number of succesful written items, negative error
     */
    int write(void *buffer,int addr,int bcount);
    
    /**
     \brief init the sensor
     \param buffer[in] initialisation opaque parameter
     \return 0 if success, error otherwise
     
     */
    int initIO(void *buffer,int sizeb);

    void driverInit(const chaos::common::data::CDataWrapper&)  ;
    /**
     \brief deinit the sensor
     \param buffer[in] initialisation opaque parameter
     \return 0 if success, error otherwise
     */
    int deinitIO();
        
};
        }}}
#endif /* defined(__ControlUnitTest__DummyDriver__) */
