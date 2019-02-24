/*
 *	AbstracSensorDriver.h
 *	!CHAOS
 *	Created by Andrea Michelotti
 *  Abstraction of a simple sensor driver
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
#ifndef __ASTRACTCAMERABRIDGE_H__
#define __ASTRACTCAMERABRIDGE_H__
#include <chaos/cu_toolkit/driver_manager/driver/AbstractDriverPlugin.h>
#include <driver/sensors/core/AbstractCameraDriver.h>


#define GETINTVALUE(x,y,LOG){\
    int32_t val;\
    LOG<<"GETTING INT PROP \""<< # x <<"\" alias:"<<y;\
    if(getNode(# x,cam,val)==0){\
    p->addInt32Value(y,(int32_t)val);} else {    BaslerScoutDriverLERR_<<"cannot read basler node \""<< #x<<"\"";\
}}

#define GETINTPERCVALUE(x,y,LOG){ \
    LOG<<"GETTING INT PERCENT PROP \""<< # x <<"\" alias:"<<y;\
    float per;\
    if(getNodeInPercentage(# x,cam,per)==0){\
    p->addDoubleValue(y,per);}}

namespace cu_driver = chaos::cu::driver_manager::driver;

namespace driver {

namespace sensor {
namespace camera{

  class CameraDriverBridge:ADD_CU_DRIVER_PLUGIN_SUPERCLASS, AbstractCameraDriver {


protected:
                 boost::mutex io_mux;

public:
    CameraDriverBridge();

    ~CameraDriverBridge();
    //! Execute a command
    cu_driver::MsgManagmentResultType::MsgManagmentResult execOpcode(cu_driver::DrvMsgPtr cmd);

};
}
}
}
#endif /* defined(__ControlUnitTest__DummyDriver__) */
