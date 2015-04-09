/*
 *	ZBSensorNode.h
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
#include <driver/sensors/models/ZBSensorNode.h>
#include <stdlib.h>
#include <string>

#include <chaos/cu_toolkit/driver_manager/driver/AbstractDriverPlugin.h>
#include <math.h>
#include <boost/lexical_cast.hpp>
#define MAX_BUF 8192

namespace cu_driver = chaos::cu::driver_manager::driver;

#define ZBSensorNodeLAPP_		LAPP_ << "[ZBSensorNode] "
#define ZBSensorNodeLDBG_		LDBG_ << "[ZBSensorNode] "
#define ZBSensorNodeLERR_		LERR_ << "[ZBSensorNode] "


ZBSensorNode::ZBSensorNode(){
   collector=NULL;
   id=-1;
}
ZBSensorNode::~ZBSensorNode() {

}
// format <com device>:nodeid:ACDDS
int ZBSensorNode::sensorInit(void *buffer,int sizeb){
    char*buf=(char*)buffer;
    char*pnt;
    int cnt=0;
    
    assert(buf);
    ZBSensorNodeLDBG_<<"Initialization string: \""<<buf<<"\"";
    pnt = strtok(buf,":\n");
    while(pnt!=0){
        cnt++;
        switch(cnt){
            case 1:
                ser = pnt;
              ZBSensorNodeLDBG_<<"Using serial:"<<ser;  
            break;
            case 2:
                id = strtoul(pnt,0,0);
                 ZBSensorNodeLDBG_<<"ID:"<<id;  
                break;
            default:
                break;
        }
        pnt=strtok(NULL,":\n");
    }
    
    if(cnt!=2){
        std::string ret("bad initialization format for ZBsensorNode valid is </dev/ttyXXX>:<sensorid>"); 
        ZBSensorNodeLERR_<<ret;
        throw chaos::CException(1,ret,"ZBSensorNode::sensorInit");
    }
   

    collector = ZBSensorCollector::getInstance();
    assert(collector);
    collector->init(ser);

    
    return 0;
}



int ZBSensorNode::sensorDeinit(){
    ZBSensorNodeLDBG_<<"deinit";
    if(collector){
        collector->deinit();
    }
    
  
    return 0;
}




