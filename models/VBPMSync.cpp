/*
 *	VBPMSync.h
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
#include <driver/sensors/models/VBPMSync.h>
#include <stdlib.h>
#include <string>

#include <chaos/cu_toolkit/driver_manager/driver/AbstractDriverPlugin.h>
#include <math.h>
#include <boost/lexical_cast.hpp>

namespace cu_driver = chaos::cu::driver_manager::driver;

#define VBPMSyncLAPP_		LAPP_ << "[VBPMSync] "
#define VBPMSyncLDBG_		LDBG_ << "[VBPMSync] "
#define VBPMSyncLERR_		LERR_ << "[VBPMSync] "


//GET_PLUGIN_CLASS_DEFINITION
//we need only to define the driver because we don't are makeing a plugin
OPEN_CU_DRIVER_PLUGIN_CLASS_DEFINITION(VBPMSync, 1.0.0,VBPMSync)
REGISTER_CU_DRIVER_PLUGIN_CLASS_INIT_ATTRIBUTE(VBPMSync, http_address/dnsname:port)
CLOSE_CU_DRIVER_PLUGIN_CLASS_DEFINITION


DEF_SENSOR_DATASET
DEF_SENSOR_CHANNEL("BPBA1001X","BPBA1001 X",chaos::DataType::Output,chaos::DataType::TYPE_DOUBLE,sizeof(double))
DEF_SENSOR_CHANNEL("BPBA1001Y","BPBA1001 Y",chaos::DataType::Output,chaos::DataType::TYPE_DOUBLE,sizeof(double))

DEF_SENSOR_CHANNEL("BPSA1001X","BPSA1001 X",chaos::DataType::Output,chaos::DataType::TYPE_DOUBLE,sizeof(double))
DEF_SENSOR_CHANNEL("BPSA1001Y","BPSA1001 Y",chaos::DataType::Output,chaos::DataType::TYPE_DOUBLE,sizeof(double))

DEF_SENSOR_CHANNEL("BPBA1002X","BPBA1002 X",chaos::DataType::Output,chaos::DataType::TYPE_DOUBLE,sizeof(double))
DEF_SENSOR_CHANNEL("BPBA1002Y","BPBA1002 Y",chaos::DataType::Output,chaos::DataType::TYPE_DOUBLE,sizeof(double))

DEF_SENSOR_CHANNEL("BPBA2001X","BPBA2001 X",chaos::DataType::Output,chaos::DataType::TYPE_DOUBLE,sizeof(double))
DEF_SENSOR_CHANNEL("BPBA2001Y","BPBA2001 Y",chaos::DataType::Output,chaos::DataType::TYPE_DOUBLE,sizeof(double))

DEF_SENSOR_CHANNEL("BPSA2001X","BPSA2001 X",chaos::DataType::Output,chaos::DataType::TYPE_DOUBLE,sizeof(double))
DEF_SENSOR_CHANNEL("BPSA2001Y","BPSA2001 Y",chaos::DataType::Output,chaos::DataType::TYPE_DOUBLE,sizeof(double))

DEF_SENSOR_CHANNEL("BPBA2002X","BPBA2002 X",chaos::DataType::Output,chaos::DataType::TYPE_DOUBLE,sizeof(double))
DEF_SENSOR_CHANNEL("BPBA2002Y","BPBA2002 Y",chaos::DataType::Output,chaos::DataType::TYPE_DOUBLE,sizeof(double))

DEF_SENSOR_CHANNEL("BPBA3001X","BPBA3001 X",chaos::DataType::Output,chaos::DataType::TYPE_DOUBLE,sizeof(double))
DEF_SENSOR_CHANNEL("BPBA3001Y","BPBA3001 Y",chaos::DataType::Output,chaos::DataType::TYPE_DOUBLE,sizeof(double))

DEF_SENSOR_CHANNEL("BPSA3001X","BPSA3001 X",chaos::DataType::Output,chaos::DataType::TYPE_DOUBLE,sizeof(double))
DEF_SENSOR_CHANNEL("BPSA3001Y","BPSA3001 Y",chaos::DataType::Output,chaos::DataType::TYPE_DOUBLE,sizeof(double))

DEF_SENSOR_CHANNEL("BPBA3002X","BPBA3002 X",chaos::DataType::Output,chaos::DataType::TYPE_DOUBLE,sizeof(double))
DEF_SENSOR_CHANNEL("BPBA3002Y","BPBA3002 Y",chaos::DataType::Output,chaos::DataType::TYPE_DOUBLE,sizeof(double))

DEF_SENSOR_CHANNEL("BPBA4001X","BPBA4001 X",chaos::DataType::Output,chaos::DataType::TYPE_DOUBLE,sizeof(double))
DEF_SENSOR_CHANNEL("BPBA4001Y","BPBA4001 Y",chaos::DataType::Output,chaos::DataType::TYPE_DOUBLE,sizeof(double))

DEF_SENSOR_CHANNEL("BPBA4002X","BPBA4002 X",chaos::DataType::Output,chaos::DataType::TYPE_DOUBLE,sizeof(double))
DEF_SENSOR_CHANNEL("BPBA4002Y","BPBA4002 Y",chaos::DataType::Output,chaos::DataType::TYPE_DOUBLE,sizeof(double))

DEF_SENSOR_CHANNEL("BPSA4001X","BPSA4001 X",chaos::DataType::Output,chaos::DataType::TYPE_DOUBLE,sizeof(double))
DEF_SENSOR_CHANNEL("BPSA4001Y","BPSA4001 Y",chaos::DataType::Output,chaos::DataType::TYPE_DOUBLE,sizeof(double))

ENDDEF_SENSOR_DATASET

//GET_PLUGIN_CLASS_DEFINITION
//we need to define the driver with alias version and a class that implement it
VBPMSync::VBPMSync(){
  INIT_SENSOR_DATASET;
   
}
//default descrutcor
VBPMSync::~VBPMSync() {

}

int VBPMSync::readChannel(void *buffer,int addr,int bcount){
    double*pnt=(double*)buffer;
    srand(time(NULL));
    int rnd=rand();
    pnt[0] = 2.5 - (rnd*1.0/RAND_MAX)*5.0;
    return 1;
}
int VBPMSync::writeChannel(void *buffer,int addr,int bcount){
    VBPMSyncLDBG_<<"writing channel :"<<addr<<" bcount:"<<bcount;
   
    return 0;
}

int VBPMSync::sensorInit(void *buffer,int sizeb){
    VBPMSyncLDBG_<<"Initialization string: \""<<buffer<<"\"";

    return 0;
}

int VBPMSync::sensorDeinit(){
    VBPMSyncLDBG_<<"deinit";
    return 1;
}




