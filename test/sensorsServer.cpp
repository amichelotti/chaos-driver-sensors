/*
*	sensorServer.cpp
*	!CHAOS
*	Created by Andrea Michelotti
*
*    	Copyright 2012 INFN, National Institute of Nuclear Physics
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

/*
to lauch control unit
BasicSensor --metadata-server mds_url:5000 --unit_server_alias alias --unit_server_enable yes

the control unit state can be checkd using the following command:
ChaosCLI --metadata-server mds_url:5000 --deviceid device_name --print-dataset [0-output, 1-input, 2-custom, 3-system]

the set of the input channel can be done (for simple format) using the following command:
ChaosCLI --metadata-server mds_url:5000 --deviceid device_name --op 9 --rt-attr-val in_1:value
*/
#include <driver/sensors/core/BasicSensor.h>
#include <driver/sensors/models/Simulator/VTemperatureDriver.h>
#include <driver/sensors/models/LucidControl/LucidOutputDriver.h>

#include <driver/sensors/models/LucidControl/LucidTemperatureDriver.h>
#include <driver/sensors/models/ZBSensor/ZBSensorCollector.h>
#include <driver/sensors/models/ZBSensor/ZBSensorNode.h>
#include <driver/sensors/models/ZBSensor/ZBSensorNodeC.h>
#include <driver/sensors/models/ZBSensor/ZBSensorNodeD.h>
#include <driver/sensors/models/ZBSensor/ZBSensorNodeS.h>
#include <string>
#ifdef CAMERA
#include <driver/sensors/models/camera/RTCameraBase.h>
#endif
#include <chaos/cu_toolkit/ChaosCUToolkit.h>

using namespace chaos;
using namespace chaos::cu;
using namespace chaos::cu::driver_manager;
using namespace ::driver::sensor;
using namespace ::driver::sensor::model;

#define OPT_CUSTOM_DEVICE_ID "device_id"

int main(int argc, char *argv[])
{
    std::string tmp_device_id;
    try {
        // initialize the control unit toolkit
        ChaosCUToolkit::getInstance()->init(argc, argv);

        // allocate the instance and inspector for driver
        REGISTER_DRIVER(,VTemperatureDriver);
        REGISTER_DRIVER(,LucidTemperatureDriver);
        REGISTER_DRIVER(,LucidOutputDriver);

        //REGISTER_DRIVER(,ZBSensorCollector);
        REGISTER_DRIVER(,ZBSensorNodeC);
        REGISTER_DRIVER(,ZBSensorNodeD);
        REGISTER_DRIVER(,ZBSensorNodeS);
        REGISTER_CU(::driver::sensor::BasicSensor);
        REGISTER_CU(::driver::sensor::camera::RTCameraBase);

        // start control unit toolkit until someone will close it
        ChaosCUToolkit::getInstance()->start();
    } catch (CException& ex) {
        DECODE_CHAOS_EXCEPTION(ex)
    } catch (program_options::error &e){
        cerr << "Unable to parse command line: " << e.what() << endl;
    } catch (...){
        cerr << "unexpected exception caught.. " << endl;
    }
    return 0;
}
