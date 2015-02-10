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
#include <driver/sensors/models/LucidTemperatureDriver.h>
#include <stdlib.h>
#include <string>

#include <chaos/cu_toolkit/driver_manager/driver/AbstractDriverPlugin.h>
#include <math.h>
#include <boost/lexical_cast.hpp>

namespace cu_driver = chaos::cu::driver_manager::driver;

#define LucidTemperatureDriverLAPP_		LAPP_ << "[LucidTemperatureDriver] "
#define LucidTemperatureDriverLDBG_		LDBG_ << "[LucidTemperatureDriver] "
#define LucidTemperatureDriverLERR_		LERR_ << "[LucidTemperatureDriver] "


//GET_PLUGIN_CLASS_DEFINITION
//we need only to define the driver because we don't are makeing a plugin
OPEN_CU_DRIVER_PLUGIN_CLASS_DEFINITION(LucidTemperatureDriver, 1.0.0,LucidTemperatureDriver)
REGISTER_CU_DRIVER_PLUGIN_CLASS_INIT_ATTRIBUTE(LucidTemperatureDriver, http_address/dnsname:port)
CLOSE_CU_DRIVER_PLUGIN_CLASS_DEFINITION



//GET_PLUGIN_CLASS_DEFINITION
//we need to define the driver with alias version and a class that implement it
LucidTemperatureDriver::LucidTemperatureDriver(){

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
  if(addr<2){

    char ch[8];
    sprintf(ch,"CH%d:",addr);
    snprintf(cmd,sizeof(cmd),"./LucidIoCtrl -d%s -tT -c%d -r",dev.c_str(),addr);
  
    std::string lista=GetStdoutFromCommand(cmd);
    pnt[addr]= atof((lista.substr((lista.find( ch )+4), (lista.size() - lista.find( ch )))).c_str());
    return 1;
    
  }
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

int LucidTemperatureDriver::getDataset(ddDataSet_t*data,int sizen){
    data[0].name="TEMP0";
    data[0].desc="Lucid Control RTD Channel0";
    data[0].type=chaos::DataType::TYPE_DOUBLE;
    data[0].dir=chaos::DataType::Output;
    data[0].maxsize=sizeof(double);
   
    data[1].name="TEMP1";
    data[1].desc="Lucid Control RTD Channel1";
    data[1].type=chaos::DataType::TYPE_DOUBLE;
    data[1].dir=chaos::DataType::Output;
    data[1].maxsize=sizeof(double);
    // add more

   return 2;

}


