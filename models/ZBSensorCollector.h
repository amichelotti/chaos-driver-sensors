/*
 *	ZBSensorCollector.h
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
#ifndef ZBSensorCollector_h
#define ZBSensorCollector_h

#include <queue>
#include <map>
#include <driver/sensors/core/AbstractSensorDriver.h>
#include <chaos/common/utility/Singleton.h>
#include <stdint.h>
#include <common/serial/core/PosixSerialComm.h>
#include <boost/thread/thread.hpp>
namespace cu_driver = chaos::cu::driver_manager::driver;
DEFINE_CU_DRIVER_DEFINITION_PROTOTYPE(ZBSensorCollector)
#define MAX_ZB_SENSOR_CHANNELS 64



typedef struct nodedata{
    int uid;
    double sensor[MAX_ZB_SENSOR_CHANNELS];
    int nsensors;
    char data[16];
    char hour[16];
    nodedata(){*data=0;*hour=0;uid=0;nsensors=0;}
} zbnodedata_t;
class ZBSensorCollector:public chaos::common::utility::Singleton<ZBSensorCollector>{

    double zbsensors_channel[MAX_ZB_SENSOR_CHANNELS];
    ::common::serial::PosixSerialComm *serial;
    int exit;
    std::map<int ,std::queue< zbnodedata_t > * > zb_queue;
    boost::thread* collector_thread;
    void updateStatus();

public:
        ZBSensorCollector();
	ZBSensorCollector(std::string);
	~ZBSensorCollector();
    void init(std::string);
    void deinit();

    zbnodedata_t getNode(int node);
  
        
};

#endif /* defined(__ControlUnitTest__DummyDriver__) */
