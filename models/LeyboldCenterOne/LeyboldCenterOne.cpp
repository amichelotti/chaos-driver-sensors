/*
 *	LeyboldCenterOne.h
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
#include <driver/sensors/models/Simulator/LeyboldCenterOne.h>
#include <stdlib.h>
#include <string>

#include <chaos/cu_toolkit/driver_manager/driver/AbstractDriverPlugin.h>
#include <math.h>
#include <boost/lexical_cast.hpp>

namespace cu_driver = chaos::cu::driver_manager::driver;
using namespace driver::sensor::model;

#define LeyboldCenterOneLAPP_		LAPP_ << "[LeyboldCenterOne] "
#define LeyboldCenterOneLDBG_		LDBG_ << "[LeyboldCenterOne] "
#define LeyboldCenterOneLERR_		LERR_ << "[LeyboldCenterOne] "


//GET_PLUGIN_CLASS_DEFINITION
//we need only to define the driver because we don't are makeing a plugin
OPEN_CU_DRIVER_PLUGIN_CLASS_DEFINITION(LeyboldCenterOne, 1.0.0,::driver::sensor::model::LeyboldCenterOne)
REGISTER_CU_DRIVER_PLUGIN_CLASS_INIT_ATTRIBUTE(::driver::sensor::model::LeyboldCenterOne, http_address/dnsname:port)
CLOSE_CU_DRIVER_PLUGIN_CLASS_DEFINITION


DEF_SENSOR_DATASET
DEF_SENSOR_CHANNEL("TEMP","temperature",chaos::DataType::Output,chaos::DataType::TYPE_DOUBLE,sizeof(double))
ENDDEF_SENSOR_DATASET

//GET_PLUGIN_CLASS_DEFINITION
//we need to define the driver with alias version and a class that implement it
LeyboldCenterOne::LeyboldCenterOne():setPoint(25),err(1),tempType(0){
  INIT_SENSOR_DATASET;
    counter=0;
    freq = 1.0;
    sinpoint=0;
    points=100.0;
}
//default descrutcor
LeyboldCenterOne::~LeyboldCenterOne() {

}

int LeyboldCenterOne::readChannel(void *buffer,int addr,int bcount){
    counter++;
    counter=counter%static_cast<int>(setPoint);
    
    switch(tempType){
        case 0:{
            double*pnt=(double*)buffer;

            srand(time(NULL));
            int rnd=rand();
            pnt[0] = (rnd*1.0/RAND_MAX)*err +setPoint;
            LeyboldCenterOneLDBG_<<"reading channel :"<<addr<<" Virtual Random Temp:"<<pnt[0];

            return 1;
        }
        case 1:{
        int rnd=rand();

            double* pnt=(double*)buffer;
            pnt[0] = counter +(rnd*1.0/RAND_MAX)*err;
            LeyboldCenterOneLDBG_<<"reading channel :"<<addr<<" Virtual Ramp Temp:"<<pnt[0];

            return 1;
        }
        case 2:{
            double*pnt=(double*)buffer;
            sinpoint++;
	    if(points>0){
          pnt[0] = setPoint+err*sin((6.28*freq)*sinpoint/points);
	    } else {
	      LeyboldCenterOneLERR_<<"reading channel :"<<addr<<" Virtual Periodic Temp, invalid number of points:"<<points;
	    }
            sinpoint=sinpoint%points;
            LeyboldCenterOneLDBG_<<"reading channel :"<<addr<<" Virtual Periodic Temp:"<<pnt[0];

            return 1;
        }
    }
    
    return 1;
    
}
int LeyboldCenterOne::writeChannel(void *buffer,int addr,int bcount){
    LeyboldCenterOneLDBG_<<"writing channel :"<<addr<<" bcount:"<<bcount;
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

int LeyboldCenterOne::sensorInit(void *buffer,int sizeb){
    LeyboldCenterOneLDBG_<<"Initialization string: \""<<buffer<<"\"";

    return 0;
}

int LeyboldCenterOne::sensorDeinit(){
    LeyboldCenterOneLDBG_<<"deinit";
    return 1;
}

void LeyboldCenterOne::driverInit(const chaos::common::data::CDataWrapper& s) throw (chaos::CException){
    LeyboldCenterOneLDBG_<<"JSON PARAMS:"<<s.getJSONString();

    if(s.hasKey("type")){
        tempType=s.getInt32Value("type");
    }
    if(s.hasKey("temp")){
        setPoint=s.getDoubleValue("temp");
        LeyboldCenterOneLDBG_<<"SET POINT:"<<setPoint;

    }
    if(s.hasKey("err")){
        err=s.getDoubleValue("err");
        LeyboldCenterOneLDBG_<<"ERROR:"<<err;

    }
}



