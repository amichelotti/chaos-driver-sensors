/*
 *	BaslerScoutDriver.h
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
#include <driver/sensors/models/cameras/Basler/BaslerScoutDriver.h>
#include <stdlib.h>
#include <string>

#include <chaos/cu_toolkit/driver_manager/driver/AbstractDriverPlugin.h>
#include <math.h>
#include <pylon/PylonIncludes.h>

#include <boost/lexical_cast.hpp>
// use the pylon driver
#include <pylon/ConfigurationEventHandler.h>

using namespace Pylon;
namespace cu_driver = chaos::cu::driver_manager::driver;
using namespace driver::sensor::model;

#define BaslerScoutDriverLAPP_		LAPP_ << "[BaslerScoutDriver] "
#define BaslerScoutDriverLDBG_		LDBG_ << "[BaslerScoutDriver] "
#define BaslerScoutDriverLERR_		LERR_ << "[BaslerScoutDriver] "


//GET_PLUGIN_CLASS_DEFINITION
//we need only to define the driver because we don't are makeing a plugin
OPEN_CU_DRIVER_PLUGIN_CLASS_DEFINITION(BaslerScoutDriver, 1.0.0,::driver::sensor::model::BaslerScoutDriver)
REGISTER_CU_DRIVER_PLUGIN_CLASS_INIT_ATTRIBUTE(::driver::sensor::model::BaslerScoutDriver, http_address/dnsname:port)
CLOSE_CU_DRIVER_PLUGIN_CLASS_DEFINITION

class CConfigurationEventPrinter : public CConfigurationEventHandler
{
                                              public:
                                              void OnAttach( CInstantCamera& /*camera*/)
{
                                              BaslerScoutDriverLDBG_ << "OnAttach event";
                                              }

                                              void OnAttached( CInstantCamera& camera)
{
                                              BaslerScoutDriverLDBG_<< "OnAttached event for device " << camera.GetDeviceInfo().GetModelName() ;
                                              }

                                              void OnOpen( CInstantCamera& camera)
{
                                              BaslerScoutDriverLDBG_<< "OnOpen event for device " << camera.GetDeviceInfo().GetModelName() ;
                                              }

                                              void OnOpened( CInstantCamera& camera)
{
                                              BaslerScoutDriverLDBG_<< "OnOpened event for device " << camera.GetDeviceInfo().GetModelName() ;
                                              }

                                              void OnGrabStart( CInstantCamera& camera)
{
                                              BaslerScoutDriverLDBG_<< "OnGrabStart event for device " << camera.GetDeviceInfo().GetModelName() ;
                                              }

                                              void OnGrabStarted( CInstantCamera& camera)
{
                                              BaslerScoutDriverLDBG_<< "OnGrabStarted event for device " << camera.GetDeviceInfo().GetModelName() ;
                                              }

                                              void OnGrabStop( CInstantCamera& camera)
{
                                              BaslerScoutDriverLDBG_<< "OnGrabStop event for device " << camera.GetDeviceInfo().GetModelName() ;
                                              }

                                              void OnGrabStopped( CInstantCamera& camera)
{
                                              BaslerScoutDriverLDBG_<< "OnGrabStopped event for device " << camera.GetDeviceInfo().GetModelName() ;
                                              }

                                              void OnClose( CInstantCamera& camera)
{
                                              BaslerScoutDriverLDBG_<< "OnClose event for device " << camera.GetDeviceInfo().GetModelName() ;
                                              }

                                              void OnClosed( CInstantCamera& camera)
{
                                              BaslerScoutDriverLDBG_<< "OnClosed event for device " << camera.GetDeviceInfo().GetModelName() ;
                                              }

                                              void OnDestroy( CInstantCamera& camera)
{
                                              BaslerScoutDriverLDBG_<< "OnDestroy event for device " << camera.GetDeviceInfo().GetModelName() ;
                                              }

                                              void OnDestroyed( CInstantCamera& /*camera*/)
{
                                              BaslerScoutDriverLDBG_<< "OnDestroyed event" ;
                                              }

                                              void OnDetach( CInstantCamera& camera)
{
                                              BaslerScoutDriverLDBG_<< "OnDetach event for device " << camera.GetDeviceInfo().GetModelName() ;
                                              }

                                              void OnDetached( CInstantCamera& camera)
{
                                              BaslerScoutDriverLDBG_<< "OnDetached event for device " << camera.GetDeviceInfo().GetModelName() ;
                                              }

                                              void OnGrabError( CInstantCamera& camera, const String_t errorMessage)
{
                                              BaslerScoutDriverLDBG_<< "OnGrabError event for device " << camera.GetDeviceInfo().GetModelName() ;
                                              BaslerScoutDriverLDBG_<< "Error Message: " << errorMessage  ;
                                              }

                                              void OnCameraDeviceRemoved( CInstantCamera& camera)
{
                                              BaslerScoutDriverLDBG_<< "OnCameraDeviceRemoved event for device " << camera.GetDeviceInfo().GetModelName() ;
                                              }
                                              };

class CCameraEventPrinter : public CCameraEventHandler
{
public:
    virtual void OnCameraEvent( CInstantCamera& camera, intptr_t userProvidedId, GenApi::INode* pNode)
    {
        BaslerScoutDriverLDBG_<<  "OnCameraEvent event for device " << camera.GetDeviceInfo().GetModelName() ;
        BaslerScoutDriverLDBG_<<  "User provided ID: " << userProvidedId ;
        BaslerScoutDriverLDBG_<<  "Event data node name: " << pNode->GetName() ;
        GenApi::CValuePtr ptrValue( pNode );
        if ( ptrValue.IsValid() )
        {
            BaslerScoutDriverLDBG_<<  "Event node data: " << ptrValue->ToString();
        }
    }
};
//GET_PLUGIN_CLASS_DEFINITION
//we need to define the driver with alias version and a class that implement it
BaslerScoutDriver::BaslerScoutDriver():camera(NULL){

    BaslerScoutDriverLDBG_<<  "Created Driver";

}
//default descrutcor
BaslerScoutDriver::~BaslerScoutDriver() {
    if(camera){
        delete camera;
    }
}

int BaslerScoutDriver::readChannel(void *buffer,int addr,int bcount){

    if(camera==NULL){
        return -1;
    }
    Pylon::CGrabResultPtr ptrGrabResult;
    BaslerScoutDriverLDBG_<<  "read channel "<<addr;

    if ( camera->WaitForFrameTriggerReady( 1000, TimeoutHandling_ThrowException)){
        camera->ExecuteSoftwareTrigger();
    }



    while(camera->GetGrabResultWaitObject().Wait( 0) == 0){
        WaitObject::Sleep( 100);
    }
    int nBuffersInQueue = 0;
    while( camera->RetrieveResult( 0, ptrGrabResult, TimeoutHandling_Return))
    {
        if ( ptrGrabResult->GetNumberOfSkippedImages())
        {
            BaslerScoutDriverLDBG_<< "Skipped " << ptrGrabResult->GetNumberOfSkippedImages() << " image.";
        }
        if (ptrGrabResult->GrabSucceeded()){
            // Access the image data.

            const uint8_t *pImageBuffer = (uint8_t *) ptrGrabResult->GetBuffer();
            //      cout << "Gray value of first pixel: " << (uint32_t) pImageBuffer[0] << endl << endl;
            int size_ret=(bcount<ptrGrabResult->GetImageSize())?bcount:ptrGrabResult->GetImageSize();
            memcpy(buffer,pImageBuffer,size_ret);
            BaslerScoutDriverLDBG_<<"SizeX: " << ptrGrabResult->GetWidth() ;
            BaslerScoutDriverLDBG_<<"SizeY: " << ptrGrabResult->GetHeight();
            BaslerScoutDriverLDBG_<<"Image Raw Size: " << size_ret ;
            return size_ret;

        }
        else
        {
            BaslerScoutDriverLERR_<< "Error: " << ptrGrabResult->GetErrorCode() << " " << ptrGrabResult->GetErrorDescription();
        }

        nBuffersInQueue++;
    }

    BaslerScoutDriverLDBG_<<"Retrieved " << nBuffersInQueue << " grab results from output queue.";

    return 0;
    
}
int BaslerScoutDriver::writeChannel(void *buffer,int addr,int bcount){
    BaslerScoutDriverLDBG_<<"writing channel :"<<addr<<" bcount:"<<bcount;

    return 0;
}

int BaslerScoutDriver::sensorInit(void *buffer,int sizeb){
    BaslerScoutDriverLDBG_<<"Initialization";
    // simple initialization
    try {
        if(camera==NULL){
            PylonInitialize();
            BaslerScoutDriverLDBG_<<  "Pylon driver initialized";
            // Create an instant camera object for the camera device found first.
            BaslerScoutDriverLDBG_<<"getting  camera informations ..";
            IPylonDevice* pdev=CTlFactory::GetInstance().CreateFirstDevice();
            if(pdev){
                BaslerScoutDriverLDBG_<<" Model:"<< pdev->GetDeviceInfo().GetModelName();
            } else {
                BaslerScoutDriverLERR_<< "cannot create instance";
                return -1;
            }
            BaslerScoutDriverLDBG_<<"creating camera";

            camera = new CInstantCamera(pdev);
            BaslerScoutDriverLDBG_<<"created camera";

            // Register the standard configuration event handler for enabling software triggering.
            // The software trigger configuration handler replaces the default configuration
            // as all currently registered configuration handlers are removed by setting the registration mode to RegistrationMode_ReplaceAll.
            camera->RegisterConfiguration( new CSoftwareTriggerConfiguration, RegistrationMode_ReplaceAll, Cleanup_Delete);
            BaslerScoutDriverLDBG_<<"trigger configuration handler installed";

            // For demonstration purposes only, add sample configuration event handlers to print out information
            // about camera use and image grabbing.
            camera->RegisterConfiguration( new CConfigurationEventPrinter, RegistrationMode_Append, Cleanup_Delete);
            BaslerScoutDriverLDBG_<<"event  handler installed";

            //camera->RegisterImageEventHandler( new CCameraEventPrinter, RegistrationMode_Append, Cleanup_Delete);
            // Print the model name of the camera.
            BaslerScoutDriverLDBG_<< "Using device " << camera->GetDeviceInfo().GetModelName();

            // The MaxNumBuffer parameter can be used to control the count of buffers
            // allocated for grabbing. The default value of this parameter is 10.
            camera->MaxNumBuffer = 15;
            BaslerScoutDriverLDBG_<<"Open camera";

            // Open the camera.
            camera->Open();
            BaslerScoutDriverLDBG_<<"Start Grabbing";

            camera->StartGrabbing( GrabStrategy_LatestImages);




        }
    } catch  (const GenericException &e){
        // Error handling.
        BaslerScoutDriverLERR_<<  "An exception occurred.";
        BaslerScoutDriverLERR_<< e.GetDescription();
        return -2;
    }


    return 0;
}

int BaslerScoutDriver::sensorDeinit(){
    BaslerScoutDriverLDBG_<<"deinit";
    return 1;
}




