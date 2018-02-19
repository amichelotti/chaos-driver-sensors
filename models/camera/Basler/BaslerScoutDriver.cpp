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
#include <driver/sensors/models/camera/Basler/BaslerScoutDriver.h>
#include <stdlib.h>
#include <string>
#include <chaos/common/data/CDataWrapper.h>
#include <chaos/cu_toolkit/driver_manager/driver/AbstractDriverPlugin.h>
#include <math.h>
#include <pylon/PylonIncludes.h>
#include <pylon/gige/ActionTriggerConfiguration.h>
#include <boost/lexical_cast.hpp>
// use the pylon driver
#include <pylon/ConfigurationEventHandler.h>

using namespace Pylon;
namespace cu_driver = chaos::cu::driver_manager::driver;
using namespace ::driver::sensor::camera;
#define BaslerScoutDriverLAPP_		LAPP_ << "[BaslerScoutDriver] "
#define BaslerScoutDriverLDBG_		LDBG_ << "[BaslerScoutDriver] "
#define BaslerScoutDriverLERR_		LERR_ << "[BaslerScoutDriver] "


//GET_PLUGIN_CLASS_DEFINITION
//we need only to define the driver because we don't are makeing a plugin
OPEN_CU_DRIVER_PLUGIN_CLASS_DEFINITION(BaslerScoutDriver, 1.0.0,::driver::sensor::camera::BaslerScoutDriver)
REGISTER_CU_DRIVER_PLUGIN_CLASS_INIT_ATTRIBUTE(::driver::sensor::camera::BaslerScoutDriver, http_address/dnsname:port)
CLOSE_CU_DRIVER_PLUGIN_CLASS_DEFINITION

void BaslerScoutDriver::driverInit(const char *initParameter) throw(chaos::CException){
    CHAOS_ASSERT(0);
}

void BaslerScoutDriver::driverInit(const chaos::common::data::CDataWrapper& json) throw(chaos::CException){
    BaslerScoutDriverLAPP_ << "Initializing  driver:"<<json.getCompliantJSONString();
    props->reset();
    props->appendAllElement((chaos::common::data::CDataWrapper&)json);
}

void BaslerScoutDriver::driverDeinit() throw(chaos::CException) {
    BaslerScoutDriverLAPP_ << "Deinit driver";

}
namespace driver{

namespace sensor{
namespace camera{
class CConfigurationEvent : public CConfigurationEventHandler
{
    ::driver::sensor::camera::BaslerScoutDriver*driver;
public:
    CConfigurationEvent(::driver::sensor::camera::BaslerScoutDriver*drv):driver(drv){};

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
        using namespace GenApi;
                    // Get the camera control object.
              INodeMap &control = camera.GetNodeMap();
              BaslerScoutDriverLDBG_<< "OnOpened event for device " << camera.GetDeviceInfo().GetModelName() ;

                    // Get the parameters for setting the image area of interest (Image AOI).
                    const CIntegerPtr width = control.GetNode("Width");
                    const CIntegerPtr height = control.GetNode("Height");
                    const CIntegerPtr offsetX = control.GetNode("OffsetX");
                    const CIntegerPtr offsetY = control.GetNode("OffsetY");
                    // Maximize the Image AOI.
                    chaos::common::data::CDataWrapper*props=driver->props;

                    if (IsWritable(offsetX) && props->hasKey("OFFSETX"))
                    {
                        offsetX->SetValue(props->getInt32Value("OFFSETX"));
                         BaslerScoutDriverLDBG_<< "setting OFFSETX " << props->getInt32Value("OFFSETX");
                    }
                    if (IsWritable(offsetY)&& props->hasKey("OFFSETY"))
                    {
                        offsetY->SetValue(props->getInt32Value("OFFSETY"));
                        BaslerScoutDriverLDBG_<< "setting OFFSETY " << props->getInt32Value("OFFSETY");

                    }
                    if (IsWritable(width)&& props->hasKey("WIDTH")){
                         width->SetValue(props->getInt32Value("WIDTH"));
                         BaslerScoutDriverLDBG_<< "setting WIDTH " << props->getInt32Value("WIDTH");

                    }
                    if (IsWritable(height)&& props->hasKey("HEIGHT")){
                         height->SetValue(props->getInt32Value("HEIGHT"));
                         BaslerScoutDriverLDBG_<< "setting HEIGHT " << props->getInt32Value("HEIGHT");

                    }
                    if (props->hasKey("PIXELFMT")){
                        // Set the pixel data format.
                        CEnumerationPtr(control.GetNode("PixelFormat"))->FromString(props->getCStringValue("PIXELFMT"));
                        BaslerScoutDriverLDBG_<< "setting Pixel Format " <<props->getCStringValue("PIXELFMT");


                    }
                    props->addInt32Value("WIDTH", (int32_t)width->GetValue());
                    props->addInt32Value("HEIGHT", (int32_t)height->GetValue());


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

class CCameraEvent : public CCameraEventHandler
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
}}}
//GET_PLUGIN_CLASS_DEFINITION
//we need to define the driver with alias version and a class that implement it
BaslerScoutDriver::BaslerScoutDriver():camera(NULL),shots(0),framebuf(NULL),fn(NULL),props(NULL),tmode(CAMERA_TRIGGER_CONTINOUS),gstrategy(CAMERA_LATEST_ONLY){

    BaslerScoutDriverLDBG_<<  "Created Driver";
    props=new chaos::common::data::CDataWrapper();

}
//default descrutcor
BaslerScoutDriver::~BaslerScoutDriver() {
    if(camera){
        delete camera;
    }
    if(props){
        delete props;
    }
}


int BaslerScoutDriver::cameraInit(void *buffer,uint32_t sizeb){
    BaslerScoutDriverLDBG_<<"Initialization";
    // simple initialization
    try {
        if(camera){
            camera->Close();
            delete camera;
            camera =NULL;
        }
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
            if(props->hasKey("TRIGGER_MODE")){
                tmode=(TriggerModes)props->getInt32Value("TRIGGER_MODE");
            }

            // Register the standard configuration event handler for enabling software triggering.
            // The software trigger configuration handler replaces the default configuration
            // as all currently registered configuration handlers are removed by setting the registration mode to RegistrationMode_ReplaceAll.
            switch(tmode){
            case (CAMERA_TRIGGER_CONTINOUS):{
                camera->RegisterConfiguration( new CAcquireContinuousConfiguration , RegistrationMode_ReplaceAll, Cleanup_Delete);
                break;
            }
            case CAMERA_TRIGGER_SINGLE:{
                camera->RegisterConfiguration( new CAcquireSingleFrameConfiguration , RegistrationMode_ReplaceAll, Cleanup_Delete);
                break;
            }
            case CAMERA_TRIGGER_SOFT:{
                camera->RegisterConfiguration( new CSoftwareTriggerConfiguration, RegistrationMode_ReplaceAll, Cleanup_Delete);

                break;
            }
            case  CAMERA_TRIGGER_HW:{
              //  camera->RegisterConfiguration( new CActionTriggerConfiguration, RegistrationMode_ReplaceAll, Cleanup_Delete);

                break;
            }
            }

            BaslerScoutDriverLDBG_<<"trigger configuration handler installed";

            // For demonstration purposes only, add sample configuration event handlers to print out information
            // about camera use and image grabbing.
            camera->RegisterConfiguration( new CConfigurationEvent(this), RegistrationMode_Append, Cleanup_Delete);
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



        }
    } catch  (const GenericException &e){
        // Error handling.
        BaslerScoutDriverLERR_<<  "An exception occurred.";
        BaslerScoutDriverLERR_<< e.GetDescription();
        return -2;
    }


    return 0;
}

int BaslerScoutDriver::cameraDeinit(){
    BaslerScoutDriverLDBG_<<"deinit";
    if(camera){
        camera->Close();
        delete camera;
        camera =NULL;
    }
    return 0;
}


int BaslerScoutDriver::startGrab(uint32_t _shots,void*_framebuf,cameraGrabCallBack _fn){
    BaslerScoutDriverLDBG_<<"Start Grabbing";
    shots=_shots;
    framebuf=_framebuf;
    fn=_fn;
    EGrabStrategy strategy;
    if(props->hasKey("GRAB_STRATEGY")){
        gstrategy=(GrabStrategy)props->getInt32Value("GRAB_STRATEGY");

    }
    switch(gstrategy){
        case CAMERA_ONE_BY_ONE:
            strategy=GrabStrategy_OneByOne;
        break;
        case CAMERA_LATEST_ONLY:
            strategy=GrabStrategy_LatestImageOnly;
        break;
    case CAMERA_LATEST:
        strategy=GrabStrategy_LatestImages;
        break;
     case CAMERA_INCOMING:
        strategy = GrabStrategy_UpcomingImage;

        break;
    }
    if(shots>0){
        camera->StartGrabbing((size_t)shots, strategy);

    } else {
        camera->StartGrabbing( strategy);
    }

}

int BaslerScoutDriver::waitGrab(uint32_t timeout_ms){
    if(camera==NULL){
        return -1;
    }
    Pylon::CGrabResultPtr ptrGrabResult;
    if(tmode> CAMERA_TRIGGER_SINGLE){
        if ( camera->WaitForFrameTriggerReady( 500, TimeoutHandling_ThrowException)){
            if(tmode == CAMERA_TRIGGER_SOFT){
                camera->ExecuteSoftwareTrigger();
            }
        }

        while(camera->GetGrabResultWaitObject().Wait( 0) == 0){
            WaitObject::Sleep( 100);
        }
    }
    int nBuffersInQueue = 0;
    while( camera->RetrieveResult( timeout_ms, ptrGrabResult, TimeoutHandling_Return))
    {
        if ( ptrGrabResult->GetNumberOfSkippedImages())
        {
            BaslerScoutDriverLDBG_<< "Skipped " << ptrGrabResult->GetNumberOfSkippedImages() << " image.";
        }
        if (ptrGrabResult->GrabSucceeded()){
            // Access the image data.

            const uint8_t *pImageBuffer = (uint8_t *) ptrGrabResult->GetBuffer();
            //      cout << "Gray value of first pixel: " << (uint32_t) pImageBuffer[0] << endl << endl;


            BaslerScoutDriverLDBG_<<"SizeX: " << ptrGrabResult->GetWidth() ;
            BaslerScoutDriverLDBG_<<"SizeY: " << ptrGrabResult->GetHeight();
            // CPylonImage target;
            // CImageFormatConverter converter;
            // converter.OutputPixelFormat=PixelType_RGB8packed;
            // converter.OutputPixelFormat=PixelType_BGR8packed;
            //  converter.OutputPixelFormat=PixelType_Mono8;
            //  converter.OutputBitAlignment=OutputBitAlignment_MsbAligned;
            //converter.Convert(target,ptrGrabResult);

            // int size_ret=(bcount<target.GetImageSize())?bcount:target.GetImageSize();
            //            memcpy(buffer,target.GetBuffer(),size_ret);

            //             int size_ret=(bcount<= ptrGrabResult->GetImageSize())?bcount: ptrGrabResult->GetImageSize();
            int size_ret=ptrGrabResult->GetImageSize();
            memcpy(framebuf,pImageBuffer,size_ret);
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
int BaslerScoutDriver::stopGrab(){
    camera->StopGrabbing();
}

int  BaslerScoutDriver::setImageProperties(uint32_t width,uint32_t height,uint32_t opencvImageType){
}

int  BaslerScoutDriver::getImageProperties(uint32_t& width,uint32_t& height,uint32_t& opencvImageType){
}


int  BaslerScoutDriver::setCameraProperty(const std::string& propname,uint32_t val){
}

int  BaslerScoutDriver::getCameraProperty(const std::string& propname,uint32_t& val){}

int  BaslerScoutDriver::getCameraProperties(std::vector<std::string >& proplist){
}
