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
#include <pylon/gige/BaslerGigEInstantCamera.h>

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
OPEN_REGISTER_PLUGIN
REGISTER_PLUGIN(::driver::sensor::camera::BaslerScoutDriver)
CLOSE_REGISTER_PLUGIN

template <typename T>
static T Adjust(T val, T minimum, T maximum, T inc)
{
    // Check the input parameters.
    if (inc <= 0)
    {
        // Negative increments are invalid.
        throw LOGICAL_ERROR_EXCEPTION("Unexpected increment %d", inc);
    }
    if (minimum > maximum)
    {
        // Minimum must not be bigger than or equal to the maximum.
        throw LOGICAL_ERROR_EXCEPTION("minimum bigger than maximum.");
    }

    // Check the lower bound.
    if (val < minimum)
    {
        return minimum;
    }

    // Check the upper bound.
    if (val > maximum)
    {
        return maximum;
    }

    // Check the increment.
    if (inc == 1)
    {
        // Special case: all values are valid.
        return val;
    }
    else
    {
        // The value must be min + (n * inc).
        // Due to the integer division, the value will be rounded down.
        return minimum + ( ((val - minimum) / inc) * inc );
    }
}

void BaslerScoutDriver::driverInit(const char *initParameter) throw(chaos::CException){
    BaslerScoutDriverLAPP_ << "Initializing  driver:"<<initParameter;
    props->reset();

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
static void setNodeInPercentage(const GenApi::CIntegerPtr& node,float percent){
    int64_t newGainRaw = node->GetMin() + ((node->GetMax() - node->GetMin())* percent);
    // Make sure the calculated value is valid
    newGainRaw = Adjust(newGainRaw,  node->GetMin(), node->GetMax(), node->GetInc());
    node->SetValue(newGainRaw);
}

static void setNodeInPercentage(const GenApi::CFloatPtr& node,float percent){
    double newGainRaw = node->GetMin() + ((node->GetMax() - node->GetMin())* percent);
    // Make sure the calculated value is valid
    newGainRaw = Adjust(newGainRaw,  node->GetMin(), node->GetMax(), node->GetInc());
    node->SetValue(newGainRaw);
}
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
        driver->propsToCamera(camera,NULL);
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

int BaslerScoutDriver::propsToCamera(CInstantCamera& camera,chaos::common::data::CDataWrapper*p){
    using namespace GenApi;
                // Get the camera control object.
          INodeMap &control = camera.GetNodeMap();
          if(p == NULL){
              p = props;
          }
          BaslerScoutDriverLDBG_<< "OnOpened event for device " << camera.GetDeviceInfo().GetModelName() ;

                // Get the parameters for setting the image area of interest (Image AOI).
                const CIntegerPtr width = control.GetNode("Width");
                const CIntegerPtr height = control.GetNode("Height");
                const CIntegerPtr offsetX = control.GetNode("OffsetX");
                const CIntegerPtr offsetY = control.GetNode("OffsetY");
#if defined( USE_USB ) || defined( USE_BCON )
                const CFloatPtr gain_node = control.GetNode("Gain");

#else
                const CIntegerPtr gain_node = control.GetNode("GainRaw");
#endif
                // Maximize the Image AOI.
             //   chaos::common::data::CDataWrapper*p=driver->props;

                if (IsWritable(offsetX) && p->hasKey("OFFSETX"))
                {
                    offsetX->SetValue(p->getInt32Value("OFFSETX"));
                     BaslerScoutDriverLDBG_<< "setting OFFSETX " << p->getInt32Value("OFFSETX");
                }
                if (IsWritable(offsetY)&& p->hasKey("OFFSETY"))
                {
                    offsetY->SetValue(p->getInt32Value("OFFSETY"));
                    BaslerScoutDriverLDBG_<< "setting OFFSETY " << p->getInt32Value("OFFSETY");

                }
                if (IsWritable(width)&& p->hasKey("WIDTH")){
                     width->SetValue(p->getInt32Value("WIDTH"));
                     BaslerScoutDriverLDBG_<< "setting WIDTH " << p->getInt32Value("WIDTH");

                }
                if (IsWritable(height)&& p->hasKey("HEIGHT")){
                     height->SetValue(p->getInt32Value("HEIGHT"));
                     BaslerScoutDriverLDBG_<< "setting HEIGHT " << p->getInt32Value("HEIGHT");

                }
                if (p->hasKey("PIXELFMT")){
                    // Set the pixel data format.
                    CEnumerationPtr(control.GetNode("PixelFormat"))->FromString(p->getCStringValue("PIXELFMT"));
                    BaslerScoutDriverLDBG_<< "setting Pixel Format " <<p->getCStringValue("PIXELFMT");
                }
                if(p->hasKey("GAIN")){
                    const CIntegerPtr gauto = control.GetNode("GainAuto");
                    if(p->getInt32Value("GAIN")<0){
                        gauto->SetValue( Basler_GigECamera::GainAutoEnums::GainAuto_Continuous);

                    } else {

                        gauto->SetValue(Basler_GigECamera::GainAutoEnums::GainAuto_Off);

                    float gain=(float)p->getInt32Value("GAIN")/100.0;
                    if(gain>1.0){
                        gain=1.0;
                    }
                    setNodeInPercentage(gain_node,gain);

                    BaslerScoutDriverLDBG_ << "Gain "<<gain<<"     : " << gain_node->GetValue() << " (Min: " << gain_node->GetMin() << "; Max: " <<  gain_node->GetMax() << "; Inc: " <<gain_node->GetInc() << ")";

                }
                }
                if(p->hasKey("SHUTTER")){
                    const CIntegerPtr exposure_node = control.GetNode("ExposureTimeRaw");

                    const CIntegerPtr gauto = control.GetNode("ExposureAuto");
                    if(p->getInt32Value("SHUTTER")<0){
                        gauto->SetValue(Basler_GigECamera::ExposureAutoEnums::ExposureAuto_Continuous);

                    } else {

                        gauto->SetValue(Basler_GigECamera::ExposureAutoEnums::ExposureAuto_Off);

                    float gain=(float)p->getInt32Value("SHUTTER")/100.0;
                    if(gain>1.0){
                        gain=1.0;
                    }
                    setNodeInPercentage(exposure_node,gain);

                    BaslerScoutDriverLDBG_ << "EXPOSURE "<<gain<<"     : " << exposure_node->GetValue() << " (Min: " << exposure_node->GetMin() << "; Max: " <<  exposure_node->GetMax() << "; Inc: " <<exposure_node->GetInc() << ")";

                }
                }
                if(p->hasKey("SHARPNESS")){
                    const CIntegerPtr sharp_node = control.GetNode("SharpnessEnhancementRaw");

                    if(p->getInt32Value("SHARPNESS")<0){

                    } else {


                    float gain=(float)p->getInt32Value("SHARPNESS")/100.0;
                    if(gain>1.0){
                        gain=1.0;
                    }
                    setNodeInPercentage(sharp_node,gain);

                    BaslerScoutDriverLDBG_ << "SHARPNESS "<<gain<<"     : " << sharp_node->GetValue() << " (Min: " << sharp_node->GetMin() << "; Max: " <<  sharp_node->GetMax() << "; Inc: " <<sharp_node->GetInc() << ")";

                }
                }
                if(p->hasKey("BRIGHTNESS")){
                    const CIntegerPtr bright_node = control.GetNode("BslBrightnessRaw");

                    if(p->getInt32Value("BRIGHTNESS")<0){

                    } else {


                    float gain=(float)p->getInt32Value("BRIGHTNESS")/100.0;
                    if(gain>1.0){
                        gain=1.0;
                    }
                    setNodeInPercentage(bright_node,gain);

                    BaslerScoutDriverLDBG_ << "BRIGHTNESS "<<gain<<"     : " << bright_node->GetValue() << " (Min: " << bright_node->GetMin() << "; Max: " <<  bright_node->GetMax() << "; Inc: " <<bright_node->GetInc() << ")";

                }
                }
                if(p->hasKey("CONTRAST")){
                    const CIntegerPtr contrast_node = control.GetNode("BslContrastRaw");

                    if(p->getInt32Value("CONTRAST")<0){

                    } else {


                    float gain=(float)p->getInt32Value("CONTRAST")/100.0;
                    if(gain>1.0){
                        gain=1.0;
                    }
                    setNodeInPercentage(contrast_node,gain);

                    BaslerScoutDriverLDBG_ << "CONTRAST "<<gain<<"     : " << contrast_node->GetValue() << " (Min: " << contrast_node->GetMin() << "; Max: " <<  contrast_node->GetMax() << "; Inc: " <<contrast_node->GetInc() << ")";

                }
                }
                p->addInt32Value("WIDTH", (int32_t)width->GetValue());
                p->addInt32Value("HEIGHT", (int32_t)height->GetValue());

    return 0;
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
    } catch(...){
        BaslerScoutDriverLERR_<<  "An exception occurred.";
        return -3;
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
    props->addInt32Value("WIDTH",width);
    props->addInt32Value("HEIGHT",height);
    return propsToCamera(*camera,props);

}

int  BaslerScoutDriver::getImageProperties(uint32_t& width,uint32_t& height,uint32_t& opencvImageType){
}


int  BaslerScoutDriver::setCameraProperty(const std::string& propname,uint32_t val){
    props->addInt32Value(propname,(int32_t)val);
    return propsToCamera(*camera,props);

}

int  BaslerScoutDriver::getCameraProperty(const std::string& propname,uint32_t& val){
    if(props->hasKey(propname)){
        val = props->getInt32Value(propname);
        return 0;
    }
    return -1;
}

int  BaslerScoutDriver::getCameraProperties(std::vector<std::string >& proplist){
}
