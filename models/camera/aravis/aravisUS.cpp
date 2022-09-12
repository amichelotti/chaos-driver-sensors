/*
*	aravisUS.cpp
*	!CHAOS
*	Created by Andrea Michelotti
*
*    	Copyright 2022 INFN, National Institute of Nuclear Physics
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


#include "AravisDriver.h"
#include "../RTCameraBase.h"
#include <arv.h>
#include <stdio.h>

#include <chaos/cu_toolkit/ChaosCUToolkit.h>
/*typedef struct {
	GMainLoop *main_loop;
	int buffer_count;
} ApplicationData;

static gboolean cancel = FALSE;
*/
int main(int argc, const char **argv)
{

    ArvCamera *camera;
	GError *error = NULL;
	int i;


	camera = arv_camera_new (NULL, &error);
    if (!ARV_IS_CAMERA (camera)) {
        printf ("No camera found%s%s\n",
			error != NULL ? ": " : "",
			error != NULL ? error->message : "");
		g_clear_error (&error);
        exit(1);
    } else {
       std::cout<<"DEVICE:"<<arv_camera_get_device_id(camera,NULL);
    }
    

    try {
        // initialize the control unit toolkit
        chaos::cu::ChaosCUToolkit::getInstance()->init(argc, argv);

        // allocate the instance and inspector for driver
        REGISTER_DRIVER(::driver::sensor::camera,AravisDriver); 


        REGISTER_CU(::driver::sensor::camera::RTCameraBase);

        chaos::cu::ChaosCUToolkit::getInstance()->start();
    } catch (chaos::CException& ex) {
        DECODE_CHAOS_EXCEPTION(ex)
    } catch (program_options::error &e){
        std::cerr << "Unable to parse command line: " << e.what() << std::endl;
    } catch (...){
        std::cerr << "unexpected exception caught.. " << std::endl;
    }
    return 0;
}
