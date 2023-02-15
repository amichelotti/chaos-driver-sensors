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
#include <driver/sensors/models/Simulator/VTemperatureDriver.h>
#include <stdlib.h>
#include <string>

#include <chaos/cu_toolkit/driver_manager/driver/AbstractDriverPlugin.h>
#include <math.h>
#include <boost/lexical_cast.hpp>

namespace cu_driver = chaos::cu::driver_manager::driver;
using namespace driver::sensor::model;

#define VTemperatureDriverLAPP_		LAPP_ << "[VTemperatureDriver] "
#define VTemperatureDriverLDBG_		LDBG_ << "[VTemperatureDriver] "
#define VTemperatureDriverLERR_		LERR_ << "[VTemperatureDriver] "


//GET_PLUGIN_CLASS_DEFINITION
//we need only to define the driver because we don't are makeing a plugin
OPEN_CU_DRIVER_PLUGIN_CLASS_DEFINITION(VTemperatureDriver, 1.0.0,::driver::sensor::model::VTemperatureDriver)
REGISTER_CU_DRIVER_PLUGIN_CLASS_INIT_ATTRIBUTE(::driver::sensor::model::VTemperatureDriver, http_address/dnsname:port)
CLOSE_CU_DRIVER_PLUGIN_CLASS_DEFINITION



//GET_PLUGIN_CLASS_DEFINITION
//we need to define the driver with alias version and a class that implement it
VTemperatureDriver::VTemperatureDriver():setPoint(25),err(1),tempType(0){
  
    counter=0;
    freq = 1.0;
    sinpoint=0;
    points=100.0;
}
//default descrutcor
VTemperatureDriver::~VTemperatureDriver() {

}

int VTemperatureDriver::read(void *buffer,int addr,int bcount){
    counter++;
    counter=counter%static_cast<int>(setPoint);
    
    switch(tempType){
        case 0:{
            double*pnt=(double*)buffer;

            srand(time(NULL));
            int rnd=rand();
            pnt[0] = (rnd*1.0/RAND_MAX)*err +setPoint;
            VTemperatureDriverLDBG_<<"reading channel :"<<addr<<" Virtual Random Temp:"<<pnt[0];
            temp=pnt[0];
            return 1;
        }
        case 1:{
        int rnd=rand();

            double* pnt=(double*)buffer;
            pnt[0] = counter +(rnd*1.0/RAND_MAX)*err;
            VTemperatureDriverLDBG_<<"reading channel :"<<addr<<" Virtual Ramp Temp:"<<pnt[0];
            temp=pnt[0];

            return 1;
        }
        case 2:{
            double*pnt=(double*)buffer;
            sinpoint++;
	    if(points>0){
          pnt[0] = setPoint+err*sin((6.28*freq)*sinpoint/points);
        temp=pnt[0];

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
int VTemperatureDriver::write(void *buffer,int addr,int bcount){
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

int VTemperatureDriver::initIO(void *buffer,int sizeb){
    VTemperatureDriverLDBG_<<"Initialization string: \""<<buffer<<"\"";

    return 0;
}

int VTemperatureDriver::deinitIO(){
    VTemperatureDriverLDBG_<<"deinit";
    return 1;
}

void VTemperatureDriver::driverInit(const chaos::common::data::CDataWrapper& s) {
    VTemperatureDriverLDBG_<<"JSON PARAMS:"<<s.getJSONString();
    temp=0;
    if(s.hasKey("type")){
        tempType=s.getInt32Value("type");
    }
    if(s.hasKey("temp")){
        setPoint=s.getDoubleValue("temp");
        VTemperatureDriverLDBG_<<"SET POINT:"<<setPoint;
        temp=setPoint;

    }
    if(s.hasKey("err")){
        err=s.getDoubleValue("err");
        VTemperatureDriverLDBG_<<"ERROR:"<<err;

    }
    createProperty("TEMP", temp,-100.0,200.0,0.0,"TEMP",[](AbstractDriver*thi,const std::string&name,
      const chaos::common::data::CDataWrapper &p) -> chaos::common::data::CDWUniquePtr {
        VTemperatureDriver *t=(VTemperatureDriver *)thi;
            chaos::common::data::CDWUniquePtr ret(new chaos::common::data::CDataWrapper());

            ret->addDoubleValue(PROPERTY_VALUE_KEY,t->temp);

            return ret;
        
      });
}



