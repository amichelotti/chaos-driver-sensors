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
camera_params_t* ret=(camera_params_t*)calloc(1,sizeof(camera_params_t));\
camera_params_t* idata=(camera_params_t*)calloc(1,sizeof(camera_params_t));\
message.opcode = op; \
 /*CameraDriverInterfaceLDBG_<<"STUB Opcode:\""<<# op<<"\" :"<<op;*/	\
message.inputData=(void*)idata;\
message.inputDataLength=sizeof(camera_params_t);\
message.resultDataLength=sizeof(camera_params_t);\
message.resultData = (void*)ret;\

#define PREPARE_RET_OP(op) \
camera_params_t* ret=(camera_params_t*)calloc(1,sizeof(camera_params_t));\
message.opcode = op; \
message.inputData=(void*)NULL;\
message.inputDataLength=0;\
message.resultDataLength=sizeof(camera_params_t);\
message.resultData = (void*)ret;\


#define SEND \
    accessor->send(&message);

#define RETURN \
    {int tmp=ret->result;free(message.inputData);free(message.resultData);return tmp;}

#define SEND_AND_RETURN \
 accessor->send(&message,chaos::common::constants::CUTimersTimeoutinMSec);			\
RETURN

#define SEND_AND_RETURN_TIM(tim) \
 accessor->send(&message,tim);			\
RETURN

#define WRITE_OP_64INT_TIM(op,ival,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
idata->alarm_mask=ival;\
SEND_AND_RETURN_TIM(timeout)


#define WRITE_OP_FLOAT_TIM(op,fval,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
idata->fvalue0=fval;\
SEND_AND_RETURN_TIM(timeout)


#define WRITE_OP_2FLOAT_TIM(op,fval0,fval1,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
idata->fvalue0=fval0;\
idata->fvalue1=fval1;\
SEND_AND_RETURN_TIM(timeout)

#define READ_OP_FLOAT_TIM(op,pfval,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
accessor->send(&message,timeout);\
*pfval = ret->fvalue0;\
int tmp=ret->result;free(ret);free(idata);return tmp;

#define READ_OP_INT_TIM(op,pival,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
accessor->send(&message,timeout);\
*pival = ret->ivalue;\
int tmp=ret->result;free(ret);free(idata);return tmp;

#define READ_OP_INT_STRING_TIM(op,pival,pstring,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
accessor->send(&message,timeout);\
*pival = ret->ivalue;\
pstring = ret->str;\
int tmp=ret->result;free(ret);free(idata);return tmp;


#define READ_OP_64INT_TIM(op,pival,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
accessor->send(&message,timeout);\
*pival = ret->alarm_mask;\
int tmp=ret->result;free(ret);free(idata);return tmp;

#define READ_OP_64INT_TIM_NORET(op,pival,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
accessor->send(&message,timeout);\
*pival = ret->alarm_mask;free(ret);free(idata);

#define READ_OP_2FLOAT_TIM(op,pfval0,pfval1,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
accessor->send(&message,timeout);\
*pfval0 = ret->fvalue0;\
*pfval1 = ret->fvalue1;\
int tmp=ret->result;free(ret);free(idata);return tmp;

#define WRITE_OP_TIM(op,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
SEND_AND_RETURN_TIM(timeout)


#define CameraDriverInterfaceLAPP_		LAPP_ << "[CameraDriverInterface] "
#define CameraDriverInterfaceLDBG_		LDBG_ << "[CameraDriverInterface] "
#define CameraDriverInterfaceLERR_		LERR_ << "[CameraDriverInterface] "


//default descrutcor
CameraDriverInterface::~CameraDriverInterface() {

}


int CameraDriverInterface::setImageProperties(int32_t width,int32_t height,int32_t opencvImageType){
    boost::mutex::scoped_lock lock(io_mux);


    PREPARE_OP(CameraDriverInterfaceOpcode_SET_IMAGE_PROP);
    idata->arg0=width;
    idata->arg1=height;
    idata->arg2=opencvImageType;
    SEND_AND_RETURN;
}

int CameraDriverInterface::getImageProperties(int32_t& width,int32_t& height,int32_t& opencvImageType){
    boost::mutex::scoped_lock lock(io_mux);


    PREPARE_OP(CameraDriverInterfaceOpcode_GET_IMAGE_PROP);
    SEND;
    width=ret->arg0;
    height=ret->arg1;
    opencvImageType=ret->arg2;
    RETURN;
}


int CameraDriverInterface::setCameraProperty(const std::string& propname,int32_t val){
    boost::mutex::scoped_lock lock(io_mux);

    PREPARE_OP(CameraDriverInterfaceOpcode_SET_PROP);
    memset(idata->property,0,MAX_PROP_STRING);

    strncpy(idata->property,propname.c_str(),MAX_PROP_STRING);
    idata->strl=strlen(idata->property);
    idata->arg0=val;
    SEND_AND_RETURN
}

int CameraDriverInterface::setCameraProperty(const std::string& propname,double val){
    boost::mutex::scoped_lock lock(io_mux);


    PREPARE_OP(CameraDriverInterfaceOpcode_SET_FPROP);
        memset(idata->property,0,MAX_PROP_STRING);

    strncpy(idata->property,propname.c_str(),MAX_PROP_STRING);
    idata->strl=strlen(idata->property);
    idata->farg=val;
   SEND_AND_RETURN
}

int CameraDriverInterface::getCameraProperty(const std::string& propname,int32_t& val){
    boost::mutex::scoped_lock lock(io_mux);


    PREPARE_OP(CameraDriverInterfaceOpcode_GET_PROP);
    memset(idata->property,0,MAX_PROP_STRING);

    strncpy(idata->property,propname.c_str(),MAX_PROP_STRING);
    
    idata->strl=strlen(idata->property);
    SEND;

    val=ret->arg0;
    RETURN
}

int CameraDriverInterface::getCameraProperty(const std::string& propname,double& val){
    boost::mutex::scoped_lock lock(io_mux);


    PREPARE_OP(CameraDriverInterfaceOpcode_GET_FPROP);
    memset(idata->property,0,MAX_PROP_STRING);

    strncpy(idata->property,propname.c_str(),MAX_PROP_STRING);
    SEND;

    val=ret->farg;
    RETURN;
}


int CameraDriverInterface::getCameraProperties(chaos::common::data::CDataWrapper& proplist){
    boost::mutex::scoped_lock lock(io_mux);

    PREPARE_RET_OP(CameraDriverInterfaceOpcode_GET_PROPERTIES);
    int sizeb=0;
    void *ptr=(void*)proplist.getBSONRawData(sizeb);
    if(ptr){
        message.inputData=malloc(sizeb);
        message.inputDataLength=sizeb;
        memcpy(message.inputData,ptr,sizeb);    
    }
    ret->str=0;
    ret->strl=0;

    SEND;
    if(ret->str && (ret->strl>0)){
        proplist.setSerializedData(ret->str);
        free(ret->str);
    }
   
    RETURN;
}

int CameraDriverInterface::startGrab(uint32_t shots,void*framebuf,cameraGrabCallBack c){
    boost::mutex::scoped_lock lock(io_mux);


    PREPARE_OP(CameraDriverInterfaceOpcode_START_GRAB);
    idata->arg0=shots;
    idata->buffer=framebuf;
    idata->fn=(void*)c;
    SEND_AND_RETURN;
}
int CameraDriverInterface::waitGrab(uint32_t timeout_ms){

    PREPARE_OP(CameraDriverInterfaceOpcode_WAIT_GRAB);
    idata->arg0=timeout_ms;
    SEND_AND_RETURN_TIM(timeout_ms);
}
int CameraDriverInterface::waitGrab(camera_buf_t**hostbuf,uint32_t timeout_ms){
    boost::mutex::scoped_lock lock(io_mux);


    PREPARE_OP(CameraDriverInterfaceOpcode_WAIT_GRABBUF);
    idata->buffer=(void*)hostbuf;
    idata->arg0=timeout_ms;
    SEND_AND_RETURN_TIM(timeout_ms);
}
int CameraDriverInterface::stopGrab(){
    boost::mutex::scoped_lock lock(io_mux);


    PREPARE_OP(CameraDriverInterfaceOpcode_STOP_GRAB);
    SEND_AND_RETURN;
}
int CameraDriverInterface::cameraInit(void *buffer,uint32_t sizeb){
        boost::mutex::scoped_lock lock(io_mux);


    PREPARE_OP(CameraDriverInterfaceOpcode_INIT);

    idata->buffer=buffer;
    idata->arg0=sizeb;
    SEND_AND_RETURN;
}

int CameraDriverInterface::cameraDeinit(){
    PREPARE_OP(CameraDriverInterfaceOpcode_DEINIT);

    SEND_AND_RETURN;

}


