/*
 *	ZBSensorNodeD.cpp
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
#include <driver/sensors/models/ZBSensor/ZBSensorNodeD.h>
#include <stdlib.h>
#include <string>

#include <chaos/cu_toolkit/driver_manager/driver/AbstractDriverPlugin.h>
#include <math.h>
#include <boost/lexical_cast.hpp>
using namespace driver::sensor::model;

DEF_SENSOR_DATASET
DEF_SENSOR_CHANNEL("TEMP","Temperatura",chaos::DataType::Output,chaos::DataType::TYPE_DOUBLE,sizeof(double))
DEF_SENSOR_CHANNEL("HUMIDITY","Umidita'",chaos::DataType::Output,chaos::DataType::TYPE_DOUBLE,sizeof(double))

ENDDEF_SENSOR_DATASET
        
 /*
 *   E 3 C 1 25567 01/04/2015 12:00
                              
Come interpretarla: (Dedicata ai sensori per misure concentrazione CO2)
                    
E 3 -> Stringa spedita dall'EndNode 3
C 1 25567 -> Il sensore per la concentrazione di CO2 numero 1 ha rilevato una concentrazione di 25567 PPM                    
01/04/2015 12:00 -> Timestamp della misurazione con risoluzione al minuto (E' possibile aggiungere anche i secondi ma non credo ce ne sia bisogno data la natura delle grandezze fisiche monitorate)  
 
 */
namespace cu_driver = chaos::cu::driver_manager::driver;

#define ZBSensorNodeDLAPP_		LAPP_ << "[ZBSensorNodeD] "
#define ZBSensorNodeDLDBG_		LDBG_ << "[ZBSensorNodeD] "
#define ZBSensorNodeDLERR_		LERR_ << "[ZBSensorNodeD] "


//GET_PLUGIN_CLASS_DEFINITION
//we need only to define the driver because we don't are makeing a plugin
OPEN_CU_DRIVER_PLUGIN_CLASS_DEFINITION(ZBSensorNodeD, 1.0.0,::driver::sensor::model::ZBSensorNodeD)
REGISTER_CU_DRIVER_PLUGIN_CLASS_INIT_ATTRIBUTE(::driver::sensor::model::ZBSensorNodeD, http_address/dnsname:port)
CLOSE_CU_DRIVER_PLUGIN_CLASS_DEFINITION



//GET_PLUGIN_CLASS_DEFINITION
//we need to define the driver with alias version and a class that implement it
ZBSensorNodeD::ZBSensorNodeD(){
    temp=0;
    humidity=0;
    updated=0;
    INIT_SENSOR_DATASET;

}
//default descrutcor
ZBSensorNodeD::~ZBSensorNodeD() {

}

int ZBSensorNodeD::readChannel(void *buffer,int addr,int bcount){
    zbnodedata_t value =collector->getNode(id | ('D'<<16));
    
    if(value.nsensors>0){
        temp = value.sensor[0];
        humidity=  value.sensor[1];
        updated=value.nsensors;
        ZBSensorNodeDLDBG_<<std::hex<<id<<std::dec<<"]Reading addr:"<<addr<<"sensors:"<<value.nsensors<<" TEMP:"<<temp<<" humidity:"<<humidity<<" date:"<<value.data<<" hour:"<<value.hour;
    } else {
        if(updated)
            updated--;
    }

    double*pnt=(double*)buffer;
    if(addr==0){
      *pnt =temp;
    } else {
      *pnt =humidity;
    }
    
    return updated;
    
}
int ZBSensorNodeD::writeChannel(void *buffer,int addr,int bcount){
    ZBSensorNodeDLDBG_<<"NOT IMPLEMENTED writing channel :"<<addr<<" bcount:"<<bcount;
   
    return 0;
}




