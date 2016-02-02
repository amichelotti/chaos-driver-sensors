/*
 *	LucidOutputDriver.h
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
#include <driver/sensors/models/LucidControl/LucidOutputDriver.h>
#include <stdlib.h>
#include <string>

#include <chaos/cu_toolkit/driver_manager/driver/AbstractDriverPlugin.h>
#include <math.h>
#include <boost/lexical_cast.hpp>

namespace cu_driver = chaos::cu::driver_manager::driver;
using namespace driver::sensor::model;

#define LucidOutputDriverLAPP_		LAPP_ << "[LucidOutputDriver] "
#define LucidOutputDriverLDBG_		LDBG_ << "[LucidOutputDriver] "
#define LucidOutputDriverLERR_		LERR_ << "[LucidOutputDriver] "


//GET_PLUGIN_CLASS_DEFINITION
//we need only to define the driver because we don't are makeing a plugin
OPEN_CU_DRIVER_PLUGIN_CLASS_DEFINITION(LucidOutputDriver, 1.0.0,::driver::sensor::model::LucidOutputDriver)
REGISTER_CU_DRIVER_PLUGIN_CLASS_INIT_ATTRIBUTE(::driver::sensor::model::LucidOutputDriver, http_address/dnsname:port)
CLOSE_CU_DRIVER_PLUGIN_CLASS_DEFINITION

DEF_SENSOR_DATASET
DEF_SENSOR_CHANNEL("OUT0","Lucid Control Voltage channel 0",chaos::DataType::Output,chaos::DataType::TYPE_DOUBLE,sizeof(double))
DEF_SENSOR_CHANNEL("OUT1","Lucid Control Voltage channel 1",chaos::DataType::Output,chaos::DataType::TYPE_DOUBLE,sizeof(double))
DEF_SENSOR_CHANNEL("OUT2","Lucid Control Voltage channel 2",chaos::DataType::Output,chaos::DataType::TYPE_DOUBLE,sizeof(double))
DEF_SENSOR_CHANNEL("OUT3","Lucid Control Voltage channel 3",chaos::DataType::Output,chaos::DataType::TYPE_DOUBLE,sizeof(double))
ENDDEF_SENSOR_DATASET


//GET_PLUGIN_CLASS_DEFINITION
//we need to define the driver with alias version and a class that implement it
LucidOutputDriver::LucidOutputDriver(){
    INIT_SENSOR_DATASET;

}
//default descrutcor
LucidOutputDriver::~LucidOutputDriver() {

}

int LucidOutputDriver::writeChannel(void *buffer,int addr,int bcount){
  
  char cmd[256];
  double*pnt=(double*)buffer;
  //add more
  if(addr<4){
    int canale,ret;
    char ch[8];
    sprintf(ch,"CH%d:",addr);
    snprintf(cmd,sizeof(cmd),"/usr/bin/LucidIoCtrl -d%s -tV -c%d -w%f",dev.c_str(),addr,*pnt);
    LucidOutputDriverLDBG_<<"LucidControl command:"<<cmd;
    return 1;
    
  }
    return 0;
}

int LucidOutputDriver::readChannel(void *buffer,int addr,int bcount){
    LucidOutputDriverLDBG_<<"read channel :"<<addr<<" Virtual Temp, not implemented";

    return 0;
}

int LucidOutputDriver::sensorInit(void *buffer,int sizeb){
  dev = (const char*)buffer;
  LucidOutputDriverLDBG_<<"Connection Device : \""<<dev<<"\"";

    return 0;
}

int LucidOutputDriver::sensorDeinit(){
    LucidOutputDriverLDBG_<<"deinit";
    return 1;
}



