/*
 *	ZBSensorNode.h
 *      Base class for ZB sensor nodes
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
#ifndef _ZBSENSORNODE_H
#define _ZBSENSORNODE_H

#include <chaos/cu_toolkit/driver_manager/driver/AbstractDriverPlugin.h>
#include <driver/sensors/core/AbstractSensorDriver.h>
#include <driver/sensors/models/ZBSensorCollector.h>

#include <chaos/common/data/DatasetDB.h>
#include <stdint.h>
#include <common/serial/core/PosixSerialComm.h>

namespace cu_driver = chaos::cu::driver_manager::driver;
#define ZBMAXSENSORS 256
class ZBSensorNode:public AbstractSensorDriver{

    protected:
    ZBSensorCollector *collector;
    std::string ser;
    int id;
  

public:
	ZBSensorNode();
	~ZBSensorNode();
  
    int sensorInit(void *buffer,int sizeb);
 
    int sensorDeinit();
        
};

#endif /* defined(__ControlUnitTest__DummyDriver__) */
