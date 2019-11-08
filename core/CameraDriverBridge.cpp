/*
 *	CameraDriverBridge.cpp
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
#include <stdlib.h>
#include <string>
#include <driver/sensors/core/CameraDriverBridge.h>
#include <chaos/common/data/CDataWrapper.h>

#include <chaos/cu_toolkit/driver_manager/driver/AbstractDriverPlugin.h>

#include <boost/lexical_cast.hpp>
#include <driver/sensors/core/CameraDriverInterface.h>

namespace cu_driver = chaos::cu::driver_manager::driver;
using namespace ::driver::sensor::camera;
#define CameraDriverBridgeLAPP_		LAPP_ << "[CameraDriverBridge] "
#define CameraDriverBridgeLDBG_		LDBG_ << "[CameraDriverBridge] "
#define CameraDriverBridgeLERR_		LERR_ << "[CameraDriverBridge] "

/*DEFAULT_CU_DRIVER_PLUGIN_CONSTRUCTOR_WITH_NS(::driver::sensor::camera, CameraDriverBridge) {
    CameraDriverBridgeLDBG_ <<" CAMERA DRIVER BRIDGE Created";

}
*/

CameraDriverBridge::CameraDriverBridge(){
        CameraDriverBridgeLDBG_ <<" CAMERA DRIVER BRIDGE Created";


}
//default destructor
CameraDriverBridge::~CameraDriverBridge() {

}



//! Execute a command
cu_driver::MsgManagmentResultType::MsgManagmentResult CameraDriverBridge::execOpcode(cu_driver::DrvMsgPtr cmd) {
    boost::mutex::scoped_lock lock(io_mux);

    cu_driver::MsgManagmentResultType::MsgManagmentResult result = cu_driver::MsgManagmentResultType::MMR_EXECUTED;
    camera_params_t *in = (camera_params_t *)cmd->inputData;
    camera_params_t *out = (camera_params_t *)cmd->resultData;
    memset(out,0,sizeof(camera_params_t));
    CameraDriverBridgeLDBG_ <<" SKELETON Opcode:"<<cmd->opcode;
    switch(cmd->opcode) {
    case CameraDriverInterfaceOpcode_SET_IMAGE_PROP:{
        int32_t width=in->arg0;
        int32_t height=in->arg1;
        int32_t opencvImageType=in->arg2;

        out->result=setImageProperties(width,height,opencvImageType);
        break;
    }
    case CameraDriverInterfaceOpcode_GET_IMAGE_PROP:{
        int32_t width;
        int32_t height;
        int32_t opencvImageType;
        out->result=getImageProperties(width,height,opencvImageType);
        out->arg0=width;
        out->arg1=height;
        out->arg2=opencvImageType;
        break;
    }

    case CameraDriverInterfaceOpcode_SET_PROP:{
        const char*pnt=in->property;
        int32_t val=in->arg0;
        out->result=setCameraProperty(pnt,val);
        break;
    }
    case CameraDriverInterfaceOpcode_SET_FPROP:{
        const char*pnt=in->property;
        double val=in->farg;
        out->result=setCameraProperty(pnt,val);
     //   free((void*)pnt);
        break;
    }
    case CameraDriverInterfaceOpcode_GET_PROP:{
        const char*pnt=in->property;
        int32_t val;
        out->result=getCameraProperty(pnt,val);
        out->arg0=val;
     //   free((void*)pnt);
        break;
    }
    case CameraDriverInterfaceOpcode_GET_FPROP:{
        const char*pnt=in->property;
        double val;
        out->result=getCameraProperty(pnt,val);
        out->farg=val;
        break;
    }
    case CameraDriverInterfaceOpcode_GET_PROPERTIES:{
        chaos::common::data::CDataWrapper props;
        int sizeb;
        out->result=getCameraProperties(props);
        const char*ptr=props.getBSONRawData(sizeb);
     
        if((sizeb>0)&& ptr){
            out->str=(char*)malloc(sizeb);
            out->strl=sizeb;
            memcpy(out->str,ptr,sizeb);
        }

        break;
    }
    case CameraDriverInterfaceOpcode_INIT:{
        out->result=cameraInit(in->buffer,in->arg0);
        break;
    }

    case CameraDriverInterfaceOpcode_DEINIT:{
        out->result=cameraDeinit();
        break;
    }
    case CameraDriverInterfaceOpcode_START_GRAB:{
        out->result=startGrab(in->arg0,in->buffer,(cameraGrabCallBack)in->fn);
        break;
    }
    case CameraDriverInterfaceOpcode_WAIT_GRAB:{
        out->result=waitGrab(in->arg0);
        break;
    }
    case CameraDriverInterfaceOpcode_WAIT_GRABBUF:{
        out->result=waitGrab((const char**)in->buffer,in->arg0);
        break;
    }
    case CameraDriverInterfaceOpcode_STOP_GRAB:{
        out->result=stopGrab();
        break;
    }
    }
    return result;
}
