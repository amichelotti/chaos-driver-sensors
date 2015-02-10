/*
 *	VTemperatureDriver.h
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
#include <driver/sensors/models/VTemperatureDriver.h>
#include <stdlib.h>
#include <string>

#include <chaos/cu_toolkit/driver_manager/driver/AbstractDriverPlugin.h>
#include <math.h>
#include <boost/lexical_cast.hpp>

namespace cu_driver = chaos::cu::driver_manager::driver;

#define VTemperatureDriverLAPP_		LAPP_ << "[VTemperatureDriver] "
#define VTemperatureDriverLDBG_		LDBG_ << "[VTemperatureDriver] "
#define VTemperatureDriverLERR_		LERR_ << "[VTemperatureDriver] "


//GET_PLUGIN_CLASS_DEFINITION
//we need only to define the driver because we don't are makeing a plugin
OPEN_CU_DRIVER_PLUGIN_CLASS_DEFINITION(VTemperatureDriver, 1.0.0,VTemperatureDriver)
REGISTER_CU_DRIVER_PLUGIN_CLASS_INIT_ATTRIBUTE(VTemperatureDriver, http_address/dnsname:port)
CLOSE_CU_DRIVER_PLUGIN_CLASS_DEFINITION

//register the two plugin
//OPEN_REGISTER_PLUGIN
//REGISTER_PLUGIN(VTemperatureDriver)
//CLOSE_REGISTER_PLUGIN



//GET_PLUGIN_CLASS_DEFINITION
//we need to define the driver with alias version and a class that implement it
VTemperatureDriver::VTemperatureDriver(){
    counter=0;
}
//default descrutcor
VTemperatureDriver::~VTemperatureDriver() {

}

int VTemperatureDriver::readChannel(void *buffer,int addr,int bcount){
    counter++;
    counter=counter%100;
    switch(addr){
        case 0:{
            double*pnt=(double*)buffer;

            srand(time(NULL));
            int rnd=rand();
            pnt[0] = (rnd*1.0/RAND_MAX)*10.0 +25;
            VTemperatureDriverLDBG_<<"reading channel :"<<addr<<" Virtual Random Temp:"<<pnt[0];

            return 1;
        }
        case 1:{
            int32_t* pnt=(int32_t*)buffer;
            pnt[0] = counter;
            VTemperatureDriverLDBG_<<"reading channel :"<<addr<<" Virtual Ramp Temp:"<<pnt[0];

            return 1;
        }
        case 2:{
            double*pnt=(double*)buffer;
            pnt[0] = 100.0*sin((6.28*counter)/100);
            VTemperatureDriverLDBG_<<"reading channel :"<<addr<<" Virtual Periodic Temp:"<<pnt[0];

            return 1;
        }
    }
    
    return 1;
    
}
int VTemperatureDriver::writeChannel(void *buffer,int addr,int bcount){
    VTemperatureDriverLDBG_<<"writing channel :"<<addr<<" Virtual Temp, not implemented";

    return 0;
}

int VTemperatureDriver::sensorInit(void *buffer,int sizeb){
    VTemperatureDriverLDBG_<<"Initialization string: \""<<buffer<<"\"";

    return 0;
}

int VTemperatureDriver::sensorDeinit(){
    VTemperatureDriverLDBG_<<"deinit";
    return 1;
}

int VTemperatureDriver::getDataset(ddDataSet_t*data,int sizen){
    data[0].name="RNDTEMP";
    data[0].desc="Virtual random temperature";
    data[0].type=chaos::DataType::TYPE_DOUBLE;
    data[0].dir=chaos::DataType::Output;
    data[0].maxsize=sizeof(double);
    
    data[1].name="RAMPTEMP";
    data[1].desc="Virtual ramp temperature";
    data[1].type=chaos::DataType::TYPE_INT32;
    data[1].dir=chaos::DataType::Output;
    data[1].maxsize=sizeof(int32_t);
    
    data[2].name="SINTEMP";
    data[2].desc="Virtual periodic temperature";
    data[2].type=chaos::DataType::TYPE_DOUBLE;
    data[2].dir=chaos::DataType::Output;
    data[2].maxsize=sizeof(double);
    return 3;

}


