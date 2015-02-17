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


DEF_SENSOR_DATASET
DEF_SENSOR_CHANNEL("RNDTEMP","Virtual random temperature",chaos::DataType::Output,chaos::DataType::TYPE_DOUBLE,sizeof(double))
DEF_SENSOR_CHANNEL("RAMPTEMP","Virtual ramp temperature",chaos::DataType::Output,chaos::DataType::TYPE_INT32,sizeof(int32_t))
DEF_SENSOR_CHANNEL("SINTEMP","Virtual periodic temperature",chaos::DataType::Output,chaos::DataType::TYPE_DOUBLE,sizeof(double))
DEF_SENSOR_CHANNEL("FREQ","Virtual periodic frequency",chaos::DataType::Input,chaos::DataType::TYPE_DOUBLE,sizeof(double))
DEF_SENSOR_CHANNEL("POINTS","Virtual periodic points",chaos::DataType::Input,chaos::DataType::TYPE_INT32,sizeof(int32_t))
ENDDEF_SENSOR_DATASET

//GET_PLUGIN_CLASS_DEFINITION
//we need to define the driver with alias version and a class that implement it
VTemperatureDriver::VTemperatureDriver(){
  INIT_SENSOR_DATASET;
    counter=0;
    freq = 1.0;
    sinpoint=0;
    points=100.0;
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
            sinpoint++;
	    if(points>0){
	      pnt[0] = points*sin((6.28*freq)*sinpoint/points);
	    } else {
	      VTemperatureDriverLERR_<<"reading channel :"<<addr<<" Virtual Periodic Temp, invalid number of points:"<<points;
	    }
            sinpoint=sinpoint%points;
            VTemperatureDriverLDBG_<<"reading channel :"<<addr<<" Virtual Periodic Temp:"<<pnt[0];

            return 1;
        }
    }
    
    return 1;
    
}
int VTemperatureDriver::writeChannel(void *buffer,int addr,int bcount){
    VTemperatureDriverLDBG_<<"writing channel :"<<addr<<" bcount:"<<bcount;
    if(addr==0){
        freq=*(double*)buffer;
        return 1;
    }
    if(addr==1){
        points=*(int32_t*)buffer;
        return 1;
    }
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




