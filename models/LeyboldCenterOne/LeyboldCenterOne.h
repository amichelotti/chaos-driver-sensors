/*
 *	LeyboldCenterOne.h
 *  driver to Read Leybold Center One Pump driver 
 *	!CHAOS
 *	Created by Andrea Michelotti
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
#ifndef LEYBOLDCENTERONE_h
#define LEYBOLDCENTERONE_h
#include <common/serial/core/SerialChannelFactory.h>
#include <chaos/cu_toolkit/driver_manager/driver/AbstractDriverPlugin.h>
#include <stdint.h>
namespace cu_driver = chaos::cu::driver_manager::driver;

/*
 driver definition
 */
DEFINE_CU_DRIVER_DEFINITION_PROTOTYPE(LeyboldCenterOne)

namespace driver {
    namespace sensor {
        namespace model {
    


class LeyboldCenterOne:ADD_CU_DRIVER_PLUGIN_SUPERCLASS{

       ::common::serial::AbstractSerialChannel_psh channel;

        
        int setContinousMode(int op);
        int sendCommand(const std::string& cmd,bool sendenq=true);

       int sendCommand(const std::string& cmd,std::string& out,bool sendenq=true);
public:
    double pressure;
    int32_t state,continuos_mode;
    int timeout;
    std::string hw_string;
    std::string gid,fwv;
    int updateValue();
	~LeyboldCenterOne();      
    void driverInit(const char *initParameter) ;

    void driverInit(const chaos::common::data::CDataWrapper&) throw(chaos::CException) ;
        
};
        }}}
#endif /* defined(__ControlUnitTest__DummyDriver__) */
