/*
 *	ZBSensorNodeC.h
 *      Class for ZB sensor nodes of class C

 *	!CHAOS
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
#ifndef _ZBSENSORNODEC_H
#define _ZBSENSORNODEC_H

#include <chaos/cu_toolkit/driver_manager/driver/AbstractDriverPlugin.h>
#include <driver/sensors/core/AbstractSensorDriver.h>
#include <chaos/common/data/DatasetDB.h>
#include <driver/sensors/models/ZBSensorNode.h>
#include <stdint.h>
namespace cu_driver = chaos::cu::driver_manager::driver;
DEFINE_CU_DRIVER_DEFINITION_PROTOTYPE(ZBSensorNodeC)

/*
 driver definition
 */
class ZBSensorNodeC:public ZBSensorNode{

    double ppm;
public:
	ZBSensorNodeC();
	~ZBSensorNodeC();
    //! Execute a command
        /**
         \brief Read a channel from the physical sensor
         \param buffer[out] destination buffer
         \param addr[in] channel address or identification
         \param bcout[in] buffer count
         \return the number of succesful read items, negative error

         */
    int readChannel(void *buffer,int addr,int bcount);
    /**
     \brief Write a channel from the physical sensor
     \param buffer[out] destination buffer
     \param addr[in] channel address or identification
     \param bcout[in] buffer count
     \return the number of succesful written items, negative error
     */
    int writeChannel(void *buffer,int addr,int bcount);
    
   
};

#endif 