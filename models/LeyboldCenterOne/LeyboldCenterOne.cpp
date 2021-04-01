/*
 *	LeyboldCenterOne.h
 *	!CHAOS
 *	Created by Andrea Michelotti.
 *
 *    	Copyright 2020 INFN, National Institute of Nuclear Physics
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
#include "LeyboldCenterOne.h"
#include <stdlib.h>
#include <string>
#include <driver/sensors/core/AbstractSensorDriver.h>
#include <chaos/cu_toolkit/driver_manager/driver/AbstractDriverPlugin.h>
#include <math.h>
#include <boost/lexical_cast.hpp>

namespace cu_driver = chaos::cu::driver_manager::driver;
using namespace driver::sensor::model;

#define LeyboldCenterOneLAPP_		LAPP_ << "[LeyboldCenterOne] "
#define LeyboldCenterOneLDBG_		LDBG_ << "[LeyboldCenterOne] - "<<__FUNCTION__
#define LeyboldCenterOneLERR_		LERR_ << "[LeyboldCenterOne] - "<<__PRETTY_FUNCTION__

#define ETX "\x3"
#define CR "\xD"
#define LF "\xA"
#define ENQ "\x5"
#define ACK "\x6"
#define NACK "\x15"

//GET_PLUGIN_CLASS_DEFINITION
//we need only to define the driver because we don't are makeing a plugin
OPEN_CU_DRIVER_PLUGIN_CLASS_DEFINITION(LeyboldCenterOne, 1.0.0,::driver::sensor::model::LeyboldCenterOne)
REGISTER_CU_DRIVER_PLUGIN_CLASS_INIT_ATTRIBUTE(::driver::sensor::model::LeyboldCenterOne, http_address/dnsname:port)
CLOSE_CU_DRIVER_PLUGIN_CLASS_DEFINITION

//GET_PLUGIN_CLASS_DEFINITION
//we need to define the driver with alias version and a class that implement it
DEFAULT_CU_DRIVER_PLUGIN_CONSTRUCTOR_WITH_NS(::driver::sensor::model, LeyboldCenterOne) {

  timeout=5000;
 
}
//default descrutcor
LeyboldCenterOne::~LeyboldCenterOne() {

}
int LeyboldCenterOne::sendCommand(const std::string& cmd,std::string&out, bool sendenq){
    char buf[256];
    int ta,ret;
    
    snprintf(buf,sizeof(buf),"%s\r\n",cmd.c_str());
    if(channel->write(buf,strlen(buf)+1,timeout)<=0){
        LeyboldCenterOneLERR_<<"Cannot send command:"<<cmd;

        return -1;
    }
    *buf=0;
    ret=channel->read(buf,sizeof(buf),"\n",timeout,&ta);
    LeyboldCenterOneLDBG_<<ret<<" answ ->"<<buf;

    if(ta){

        LeyboldCenterOneLERR_<<"Timeout sending command";
        return ret;
    }
    const char* check_ans=ACK CR LF;
    if(strstr(buf,check_ans)){
        if(sendenq){
            const char * buf=ENQ;
            if(channel->write((void*)buf,1,timeout)<=0){
                LeyboldCenterOneLERR_<<"Cannot send <ENC>:"<<cmd;
                return -22;
            } else {
                char buffer[256];
                int timeout_arised=0;
                int ret=channel->read(&buf,sizeof(buffer),"\n",timeout,&timeout_arised);
                if(ret>0){
                    out=buffer;
                    LeyboldCenterOneLDBG_<<"ANSW:"<<buffer;

                } else {
                    out="";
                }
            }
        }
        return 0;
    }
    check_ans=NACK CR LF;
    if(strstr(buf,check_ans)){
        LeyboldCenterOneLERR_<<"Replies BAD";

        return 10;
    }

    LeyboldCenterOneLERR_<<"bad answer:"<<buf;
    return -3;
}

int LeyboldCenterOne::updateValue(){
    char buffer[256];
    int ret;
    int timeout_arised=0;
    if(continuos_mode>=0){
        ret=channel->read(buffer,sizeof(buffer),"\n",timeout,&timeout_arised);
        if(ret>0){
            LeyboldCenterOneLDBG_<<":"<<buffer;
            hw_string=buffer;
            if(sscanf(buffer,"%d,%lf",&state,&pressure)==2){
                return 0;
            }
        }
    } else {
        std::string ans;
        ret=sendCommand("PR1",ans,true);
        if(ret==0){
            hw_string=ans;
            LeyboldCenterOneLDBG_<<"PR1:"<<ans;

             if(sscanf(ans.c_str(),"%d,%lf",&state,&pressure)==2){
                 return 0;
            }
        }
    }
    
    return ret;

}

int LeyboldCenterOne::sendCommand(const std::string& cmd,bool sendenq){
    std::string tmp;
    return sendCommand(cmd,tmp,sendenq);
}

int LeyboldCenterOne::setContinousMode(int op){
    std::stringstream ss;
    ss<<"COM,"<<op;
    return sendCommand(ss.str(),false);

}
void LeyboldCenterOne::driverInit(const char *initParameter) {

    throw chaos::CException(-11,"Should provide JSON params",__PRETTY_FUNCTION__);
    
}

void LeyboldCenterOne::driverInit(const chaos::common::data::CDataWrapper& s) throw (chaos::CException){
    LeyboldCenterOneLDBG_<<"JSON PARAMS:"<<s.getJSONString();


   channel=::common::serial::SerialChannelFactory::getChannel(s);

    if(channel.get()==NULL){
        throw chaos::CException(-10,"Cannot open serial channel",__PRETTY_FUNCTION__);
    }
    state=0;
    continuos_mode=-1;
    std::string ret;
    channel->init();
    if(sendCommand("RES,1",ret,true)){
        LeyboldCenterOneLERR_<<"Resetting "<<ret;
    } else {
        LeyboldCenterOneLDBG_<<"Reset OK: "<<ret;
    }
    //firmaware
    if(sendCommand("PNR",fwv,true)){
        LeyboldCenterOneLERR_<<"Getting FW version "<<fwv;
    } else {
        LeyboldCenterOneLDBG_<<"Firmware: "<<fwv;
    }
    //gauge identification
    if(sendCommand("TID",gid,true)){
        LeyboldCenterOneLERR_<<"Getting Gauge "<<gid;
    } else {
        LeyboldCenterOneLDBG_<<"Gauge: "<<gid;
    }
    // set continuos mode
    if(setContinousMode(0)){
        LeyboldCenterOneLERR_<<"continuos mode "<<continuos_mode;
    } else {
        continuos_mode=0;
    }
     createProperty("COM", continuos_mode,0,2,1,"",NULL,[](AbstractDriver*thi,const std::string&name,
      const chaos::common::data::CDataWrapper &p) -> chaos::common::data::CDWUniquePtr {
        LeyboldCenterOne *t=(LeyboldCenterOne *)thi;
            int32_t val=p.getInt32Value("value");
            if(t->setContinousMode(val)==0){
                LeyboldCenterOneLDBG_<<"continuos mode "<<val;
                t->continuos_mode=val;
                return p.clone();
            }       
            return chaos::common::data::CDWUniquePtr();
        
      });

    createProperty("State", state,0,0x20,1,SENSOR_STATE_KEY,[](AbstractDriver*thi,const std::string&name,
      const chaos::common::data::CDataWrapper &p) -> chaos::common::data::CDWUniquePtr {
        LeyboldCenterOne *t=(LeyboldCenterOne *)thi;
            int32_t value=SENSOR_STATE_OK;
            t->updateValue();

            LeyboldCenterOneLDBG_<< "HW STATE:"<<t->state;
            chaos::common::data::CDWUniquePtr ret(new chaos::common::data::CDataWrapper());
            switch(t->state){
                case 1:
                    value|=SENSOR_STATE_UNDERFLOW;
                    break;
                case 2:
                    value|=SENSOR_STATE_OVERFLOW;
                    break;
                case 3:
                    value|=SENSOR_STATE_ERROR;
                    break;
                case 4:
                    value|=SENSOR_STATE_OFF;
                    break;
                case 5:
                    value|=SENSOR_STATE_NOSENSOR;
                    break;
                case 6:
                case 7:
                    value|=SENSOR_STATE_IDERROR;
                    break;
                default:
                    value=SENSOR_STATE_OK;
            }

            ret->addInt32Value("value",value);

            return ret;
        
      });


  createProperty("Pressure", pressure,0.0,10000.0,0.0,"PRESSURE",[](AbstractDriver*thi,const std::string&name,
      const chaos::common::data::CDataWrapper &p) -> chaos::common::data::CDWUniquePtr {
        LeyboldCenterOne *t=(LeyboldCenterOne *)thi;
            chaos::common::data::CDWUniquePtr ret(new chaos::common::data::CDataWrapper());

            ret->addDoubleValue("value",t->pressure);

            return ret;
        
      });



}



