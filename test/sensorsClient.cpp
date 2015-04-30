
/*
 *	sensorClient.cpp
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

#include <stdio.h>

#include <chaos/ui_toolkit/ChaosUIToolkit.h>
#include <chaos/ui_toolkit/LowLevelApi/LLRpcApi.h>
#include <chaos/ui_toolkit/HighLevelApi/HLDataApi.h>

using namespace chaos;
using namespace chaos::ui;

void print_state(CUStateKey::ControlUnitState state) {
  switch (state) {
    case CUStateKey::INIT:
    std::cout << "Initialized" << std::endl;
    break;
    case CUStateKey::START:
    std::cout << "Started" << std::endl;
    break;
    case CUStateKey::STOP:
    std::cout << "Stopped" << std::endl;
    break;
    case CUStateKey::DEINIT:
    std::cout << "Deinitilized" << std::endl;
    break;
  }
}


int main (int argc, char* argv[] ) {
  int err = 0;
  std::string attribute_value_tmp_str;
  CUStateKey::ControlUnitState device_state;
  try{
    //init UIToolkit client
    ChaosUIToolkit::getInstance()->init(argc, argv);

    DeviceController *controller = HLDataApi::getInstance()->getControllerForDeviceID("device_name", 40000);
    if(!controller) return -1;

    //init device
    std::cout << "Init the device" << std::endl;
    err = controller->initDevice();
    if(err == ErrorCode::EC_TIMEOUT) return -1;
    print_state(device_state);
    sleep(2);

    //check the state
    err = controller->getState(device_state);
    if(err == ErrorCode::EC_TIMEOUT) return -1;
    print_state(device_state);
    sleep(2);

    //start the device
    std::cout << "Start the device" << std::endl;
    err = controller->startDevice();
    sleep(2);

    //check the state
    err = controller->getState(device_state);
    if(err == ErrorCode::EC_TIMEOUT) return -1;
    print_state(device_state);
    sleep(2);

    //print all dataset
    controller->fetchCurrentDatatasetFromDomain((DatasetDomain)0);
    if(controller->getCurrentDatasetForDomain((DatasetDomain)0) != NULL) {
      std::cout << controller->getCurrentDatasetForDomain((DatasetDomain)0)->getJSONString() <<std::endl;
    }

    controller->fetchCurrentDatatasetFromDomain((DatasetDomain)1);
    if(controller->getCurrentDatasetForDomain((DatasetDomain)1) != NULL) {
      std::cout << controller->getCurrentDatasetForDomain((DatasetDomain)1)->getJSONString() <<std::endl;
    }

    controller->fetchCurrentDatatasetFromDomain((DatasetDomain)2);
    if(controller->getCurrentDatasetForDomain((DatasetDomain)2) != NULL) {
      std::cout << controller->getCurrentDatasetForDomain((DatasetDomain)2)->getJSONString() <<std::endl;
    }

    controller->fetchCurrentDatatasetFromDomain((DatasetDomain)3);
    if(controller->getCurrentDatasetForDomain((DatasetDomain)3) != NULL) {
      std::cout << controller->getCurrentDatasetForDomain((DatasetDomain)3)->getJSONString() <<std::endl;
    }

    //set schedule time to 250 milliseconds
    std::cout << "set run schedule to 250 milliseconds" << std::endl;
    err = controller->setScheduleDelay(250000);
    sleep(2);

    //set a value on input parameter without waith the ack
    std::cout << "set in_1 to " << 20 << std::endl;
    err = controller->setAttributeToValue("in_1", "20", true);
    sleep(2);

    //!update output dataset
    controller->fetchCurrentDatatasetFromDomain((DatasetDomain)0);
    if(controller->getCurrentDatasetForDomain((DatasetDomain)0) != NULL) {
      std::cout << controller->getCurrentDatasetForDomain((DatasetDomain)0)->getJSONString() <<std::endl;
    }

    controller->fetchCurrentDatatasetFromDomain((DatasetDomain)1);
    if(controller->getCurrentDatasetForDomain((DatasetDomain)1) != NULL) {
      std::cout << controller->getCurrentDatasetForDomain((DatasetDomain)1)->getJSONString() <<std::endl;
    }

    controller->fetchCurrentDatatasetFromDomain((DatasetDomain)2);
    if(controller->getCurrentDatasetForDomain((DatasetDomain)2) != NULL) {
      std::cout << controller->getCurrentDatasetForDomain((DatasetDomain)2)->getJSONString() <<std::endl;
    }

    controller->fetchCurrentDatatasetFromDomain((DatasetDomain)3);
    if(controller->getCurrentDatasetForDomain((DatasetDomain)3) != NULL) {
      std::cout << controller->getCurrentDatasetForDomain((DatasetDomain)3)->getJSONString() <<std::endl;
    }
    sleep(2);

    //! stop the device
    std::cout << "Stop the device" << std::endl;
    err = controller->stopDevice();
    err = controller->getState(device_state);
    if(err == ErrorCode::EC_TIMEOUT) return -1;
    print_state(device_state);
    sleep(2);

    //! deinit the device
    std::cout << "Deinit the device" << std::endl;
    err = controller->deinitDevice();
    err = controller->getState(device_state);
    if(err == ErrorCode::EC_TIMEOUT) return -1;
    print_state(device_state);
    sleep(2);
    } catch (CException& e) {
        std::cerr << e.errorCode << " - "<< e.errorDomain << " - " << e.errorMessage << std::endl;
    }
    //deinit the toolkit
    try {
        //! [UIToolkit Deinit]
        ChaosUIToolkit::getInstance()->deinit();
        //! [UIToolkit Deinit]
    } catch (CException& e) {
        std::cerr << e.errorCode << " - "<< e.errorDomain << " - " << e.errorMessage << std::endl;
    }

    return 0;
}
