/*
 *	CameraDriverInterface.cpp
 *	!CHAOS
 *	Created by Andrea Michelotti
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
#include "CameraDriverInterface.h"



using namespace ::driver::sensor::camera;


#define PREPARE_OP(op) \
camera_params_t ret;\
camera_params_t idata;\
message.opcode = op; \
message.inputData=(void*)&idata;\
message.inputDataLength=sizeof(camera_params_t);\
message.resultDataLength=sizeof(camera_params_t);\
message.resultData = (void*)&ret;\

#define SEND_AND_RETURN \
 accessor->send(&message,100);			\
return ret.result;

#define SEND \
    accessor->send(&message,100);


#define WRITE_OP_64INT_TIM(op,ival,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
idata.alarm_mask=ival;\
accessor->send(&message);\
return ret.result;

#define WRITE_OP_FLOAT_TIM(op,fval,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
idata.fvalue0=fval;\
 accessor->send(&message,100);			\
return ret.result;

#define WRITE_OP_2FLOAT_TIM(op,fval0,fval1,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
idata.fvalue0=fval0;\
idata.fvalue1=fval1;\
accessor->send(&message);\
return ret.result;

#define READ_OP_FLOAT_TIM(op,pfval,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
accessor->send(&message);\
*pfval = ret.fvalue0;\
return ret.result;

#define READ_OP_INT_TIM(op,pival,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
accessor->send(&message);\
*pival = ret.ivalue;\
return ret.result;

#define READ_OP_INT_STRING_TIM(op,pival,pstring,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
accessor->send(&message);\
*pival = ret.ivalue;\
pstring = ret.str;\
return ret.result;

#define READ_OP_INT_TIM(op,pival,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
accessor->send(&message);\
*pival = ret.ivalue;\
return ret.result;

#define READ_OP_64INT_TIM(op,pival,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
accessor->send(&message);\
*pival = ret.alarm_mask;\
return ret.result;

#define READ_OP_64INT_TIM_NORET(op,pival,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
accessor->send(&message);\
*pival = ret.alarm_mask;


#define READ_OP_2FLOAT_TIM(op,pfval0,pfval1,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
accessor->send(&message);\
*pfval0 = ret.fvalue0;\
*pfval1 = ret.fvalue1;\
return ret.result;

#define WRITE_OP_TIM(op,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
accessor->send(&message);\
return ret.result;



#define CameraDriverInterfaceLAPP_		LAPP_ << "[CameraDriverInterface] "
#define CameraDriverInterfaceLDBG_		LDBG_ << "[CameraDriverInterface] "
#define CameraDriverInterfaceLERR_		LERR_ << "[CameraDriverInterface] "


//default descrutcor
CameraDriverInterface::~CameraDriverInterface() {

}


int CameraDriverInterface::setImageProperties(uint32_t width,uint32_t height,uint32_t opencvImageType){
    PREPARE_OP(CameraDriverInterfaceOpcode_SET_IMAGE_PROP);
    idata.arg0=width;
    idata.arg1=height;
    idata.arg2=opencvImageType;
    SEND_AND_RETURN;
}

int CameraDriverInterface::getImageProperties(uint32_t& width,uint32_t& height,uint32_t& opencvImageType){
    PREPARE_OP(CameraDriverInterfaceOpcode_GET_IMAGE_PROP);
    SEND;
    width=ret.arg0;
    height=ret.arg1;
    opencvImageType=ret.arg2;
    return ret.result;
}


int CameraDriverInterface::setCameraProperty(const std::string& propname,uint32_t val){
    PREPARE_OP(CameraDriverInterfaceOpcode_SET_PROP);
    idata.str=strdup(propname.c_str());
    idata.strl=propname.size();
    idata.arg0=val;
    SEND_AND_RETURN;
}

int CameraDriverInterface::getCameraProperty(const std::string& propname,uint32_t& val){
    PREPARE_OP(CameraDriverInterfaceOpcode_GET_PROP);
    idata.str=(char*)propname.c_str();
    idata.strl=propname.size();
    SEND;

    val=ret.arg0;
    return ret.result;
}


int CameraDriverInterface::getCameraProperties(std::vector<std::string >& proplist){
    PREPARE_OP(CameraDriverInterfaceOpcode_GET_PROPERTIES);
    int cnt;
    char* start_str=0;
    SEND;
    if(ret.str){
    start_str=ret.str;
    for(cnt=0;cnt<ret.strl;cnt++){
        char *pnt=(ret.str + cnt);
        if((*pnt)==0){
            std::string a;
            a.assign(start_str);
            proplist.push_back(a);
            start_str=(pnt+1);
        }
    }
    free(ret.str);
}
    return ret.result;
}

int CameraDriverInterface::startGrab(uint32_t shots,void*framebuf,cameraGrabCallBack c){
    PREPARE_OP(CameraDriverInterfaceOpcode_START_GRAB);
    idata.arg0=shots;
    idata.buffer=framebuf;
    idata.fn=(void*)c;
    SEND_AND_RETURN;
}
int CameraDriverInterface::waitGrab(uint32_t timeout_ms){
    PREPARE_OP(CameraDriverInterfaceOpcode_WAIT_GRAB);
    idata.arg0=timeout_ms;
    SEND_AND_RETURN;
}

int CameraDriverInterface::stopGrab(){
    PREPARE_OP(CameraDriverInterfaceOpcode_STOP_GRAB);
    SEND_AND_RETURN;
}
int CameraDriverInterface::cameraInit(void *buffer,uint32_t sizeb){
    PREPARE_OP(CameraDriverInterfaceOpcode_INIT);
    idata.buffer=buffer;
    idata.arg0=sizeb;
    SEND_AND_RETURN;
}

int CameraDriverInterface::cameraDeinit(){
    PREPARE_OP(CameraDriverInterfaceOpcode_DEINIT);

    SEND_AND_RETURN;

}
