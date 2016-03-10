/*
 *	LucidTemperatureDriver.h
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
#include <driver/sensors/models/LucidControl/LucidTemperatureDriver.h>
#include <stdlib.h>
#include <string>

#include <math.h>
#include <boost/lexical_cast.hpp>

namespace cu_driver = chaos::cu::driver_manager::driver;

#define LucidTemperatureDriverLAPP_		LAPP_ << "[LucidTemperatureDriver] "
#define LucidTemperatureDriverLDBG_		LDBG_ << "[LucidTemperatureDriver] "
#define LucidTemperatureDriverLERR_		LERR_ << "[LucidTemperatureDriver] "
using namespace ::driver::sensor::model;

//GET_PLUGIN_CLASS_DEFINITION
//we need only to define the driver because we don't are makeing a plugin
OPEN_CU_DRIVER_PLUGIN_CLASS_DEFINITION(LucidTemperatureDriver, 1.0.0,::driver::sensor::model::LucidTemperatureDriver)
REGISTER_CU_DRIVER_PLUGIN_CLASS_INIT_ATTRIBUTE(::driver::sensor::model::LucidTemperatureDriver, http_address/dnsname:port)
CLOSE_CU_DRIVER_PLUGIN_CLASS_DEFINITION

DEF_SENSOR_DATASET
DEF_SENSOR_CHANNEL("TEMP0","Lucid Control RTD Channel0",chaos::DataType::Output,chaos::DataType::TYPE_DOUBLE,sizeof(double))
DEF_SENSOR_CHANNEL("TEMP1","Lucid Control RTD Channel1",chaos::DataType::Output,chaos::DataType::TYPE_DOUBLE,sizeof(double))
DEF_SENSOR_CHANNEL("TEMP2","Lucid Control RTD Channel2",chaos::DataType::Output,chaos::DataType::TYPE_DOUBLE,sizeof(double))
DEF_SENSOR_CHANNEL("TEMP3","Lucid Control RTD Channel3",chaos::DataType::Output,chaos::DataType::TYPE_DOUBLE,sizeof(double))

ENDDEF_SENSOR_DATASET


//GET_PLUGIN_CLASS_DEFINITION
//we need to define the driver with alias version and a class that implement it
LucidTemperatureDriver::LucidTemperatureDriver(){
    INIT_SENSOR_DATASET;

}
std::string GetStdoutFromCommand(std::string cmd) {
  
  std::string data;
  FILE * stream;
  const int max_buffer = 256;
  char buffer[max_buffer];
  cmd.append(" 2>&1");
  
  stream = popen(cmd.c_str(), "r");
  if (stream) {
    while (!feof(stream))
      if (fgets(buffer, max_buffer, stream) != NULL) data.append(buffer);
    pclose(stream);
  }
  return data;
}
//default descrutcor
LucidTemperatureDriver::~LucidTemperatureDriver() {

}

int LucidTemperatureDriver::readChannel(void *buffer,int addr,int bcount){
  
  char cmd[256];
  double*pnt=(double*)buffer;
  //add more
  if(addr<4){
    int canale,ret;
    char ch[8];
    sprintf(ch,"CH%d:",addr);
    snprintf(cmd,sizeof(cmd),"/usr/bin/LucidIoCtrl -d%s -tT -c%d -r",dev.c_str(),addr);
    LucidTemperatureDriverLDBG_<<"LucidControl command:"<<cmd;
    std::string lista=GetStdoutFromCommand(cmd);
    LucidTemperatureDriverLDBG_<<"LucidControl answer:"<<lista;
    ///    pnt[addr]= atof((lista.substr((lista.find( ch )+4), (lista.size() - lista.find( ch )))).c_str());
    if((ret=sscanf(lista.c_str(),"CH%d:%lf",&canale,pnt))==2){
      LucidTemperatureDriverLDBG_<<"LucidControl channel:"<<canale<<" temp:"<<*pnt;
    } else {
      LucidTemperatureDriverLERR_<<"LucidControl error parsing";
      return 0;
    }
    
    return 1;
    
  }
    return 0;
}

int LucidTemperatureDriver::writeChannel(void *buffer,int addr,int bcount){
    LucidTemperatureDriverLDBG_<<"writing channel :"<<addr<<" Virtual Temp, not implemented";

    return 0;
}

int LucidTemperatureDriver::sensorInit(void *buffer,int sizeb){
  dev = (const char*)buffer;
  LucidTemperatureDriverLDBG_<<"Connection Device : \""<<dev<<"\"";

    return 0;
}

int LucidTemperatureDriver::sensorDeinit(){
    LucidTemperatureDriverLDBG_<<"deinit";
    return 1;
}



