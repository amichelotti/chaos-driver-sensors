/*
 *	IDSGEXXDriver.h
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
#include "IDSGEXXDriver.h"
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
#define IDSGEXXDriverLAPP_		LAPP_ << "[IDSGEXXDriver] "
#define IDSGEXXDriverLDBG_		LDBG_ << "[IDSGEXXDriver:"<<__PRETTY_FUNCTION__<<"]"
#define IDSGEXXDriverLERR_		LERR_ << "[IDSGEXXDriver:"<<__PRETTY_FUNCTION__<<"]"


//GET_PLUGIN_CLASS_DEFINITION
//we need only to define the driver because we don't are makeing a plugin
OPEN_CU_DRIVER_PLUGIN_CLASS_DEFINITION(IDSGEXXDriver, 1.0.0,::driver::sensor::camera::IDSGEXXDriver)
REGISTER_CU_DRIVER_PLUGIN_CLASS_INIT_ATTRIBUTE(::driver::sensor::camera::IDSGEXXDriver, http_address/dnsname:port)
CLOSE_CU_DRIVER_PLUGIN_CLASS_DEFINITION
OPEN_REGISTER_PLUGIN
REGISTER_PLUGIN(::driver::sensor::camera::IDSGEXXDriver)
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

void IDSGEXXDriver::driverInit(const char *initParameter) throw(chaos::CException){
    IDSGEXXDriverLAPP_ << "Initializing  driver:"<<initParameter;
    props->reset();
    if(initializeCamera(*props)!=0){
        throw chaos::CException(-3,"cannot initialize camera ",__PRETTY_FUNCTION__);

    }
}

void IDSGEXXDriver::driverInit(const chaos::common::data::CDataWrapper& json) throw(chaos::CException){
    IDSGEXXDriverLAPP_ << "Initializing  driver:"<<json.getCompliantJSONString();
    props->reset();
    props->appendAllElement((chaos::common::data::CDataWrapper&)json);
    if(initializeCamera(*props)!=0){
        throw chaos::CException(-1,"cannot initialize camera "+json.getCompliantJSONString(),__PRETTY_FUNCTION__);
    }


}

void IDSGEXXDriver::driverDeinit() throw(chaos::CException) {
    IDSGEXXDriverLAPP_ << "Deinit driver";

}
using namespace GenApi;

namespace driver{

namespace sensor{
namespace camera{

static int setNode(const std::string& node_name,CInstantCamera& camera,int64_t val){
    try{
        INodeMap &control = camera.GetNodeMap();
        GenApi::CIntegerPtr node=control.GetNode(node_name.c_str());
        if(IsWritable(node)){
            node->SetValue(val);
        } else {
            IDSGEXXDriverLERR_<<  "Node:"<<node_name<< " is not writable";
            return -100;

        }
    }catch  (const GenericException &e){
        // Error handling.
        IDSGEXXDriverLERR_<<  "An exception occurred during set of Node:"<<node_name;
        IDSGEXXDriverLERR_<< e.GetDescription();
        return -3;
    } catch(...){
        IDSGEXXDriverLERR_<<  "An exception occurre during set of Node:"<<node_name;
        return -2;
    }
    return 0;
}

static int setNode(const std::string& node_name,CInstantCamera& camera,double val){
    try{
        INodeMap &control = camera.GetNodeMap();
        GenApi::CFloatPtr node=control.GetNode(node_name.c_str());
         node->SetValue(val);
      /*  if(IsWritable(node)){
            node->SetValue(val);
        } else {
            IDSGEXXDriverLERR_<<  "Node:"<<node_name<< " is not writable";
            return -100;

        }*/
    }catch  (const GenericException &e){
        // Error handling.
        IDSGEXXDriverLERR_<<  "An exception occurred during SET of Node:"<<node_name;
        IDSGEXXDriverLERR_<< e.GetDescription();
        return -3;
    } catch(...){
        IDSGEXXDriverLERR_<<  "An exception occurre during SET of Node:"<<node_name;
        return -2;
    }
    return 0;
    return 0;
}
static int setNodeInPercentage(const std::string& node_name,CInstantCamera& camera,float percent){
    try {
        INodeMap &control = camera.GetNodeMap();
        GenApi::CIntegerPtr node=control.GetNode(node_name.c_str());
        int64_t min,max,inc;
        min=node->GetMin() ;
        max=node->GetMax();
        inc=node->GetInc();
        int64_t newGainRaw = min+ ((max - min)* percent);
        // Make sure the calculated value is valid
        IDSGEXXDriverLDBG_<<"MIN:"<<min<<" MAX:"<<max<<" INC:"<<inc<<" VALUE:"<<newGainRaw<< " percent:"<<percent*100;

        newGainRaw = Adjust(newGainRaw,  min, max, inc);
        node->SetValue(newGainRaw);
    } catch  (const GenericException &e){
        // Error handling.
        IDSGEXXDriverLERR_<<  "An exception occurred during SET of Node:"<<node_name;
        IDSGEXXDriverLERR_<< e.GetDescription();
        return -3;
    } catch(...){
        IDSGEXXDriverLERR_<<  "An exception occurre during SET of Node:"<<node_name;
        return -2;
    }
    return 0;
}

static int getNode(const std::string& node_name,CInstantCamera& camera,int32_t& percent){
    try {
        INodeMap &control = camera.GetNodeMap();
        GenApi::CIntegerPtr node=control.GetNode(node_name.c_str());
        percent=-1;
        percent=node->GetValue();

        IDSGEXXDriverLDBG_<<"VAL:"<<percent;
    } catch  (const GenericException &e){
        // Error handling.
        IDSGEXXDriverLERR_<<  "An exception occurred during GET of Node:"<<node_name;
        IDSGEXXDriverLERR_<< e.GetDescription();
        return -3;
    } catch(...){
        IDSGEXXDriverLERR_<<  "An exception occurre during GET of Node:"<<node_name;
        return -2;
    }
    return 0;
}
static int getNodeInPercentage(const std::string& node_name,CInstantCamera& camera,float& percent){
    int64_t min,max,inc,val;
    try {
        INodeMap &control = camera.GetNodeMap();
        GenApi::CIntegerPtr node=control.GetNode(node_name.c_str());
        percent=-1;
        min=node->GetMin() ;
        max=node->GetMax();
        inc=node->GetInc();
        val=node->GetValue();
        if((max-min)>0){
            percent = (val*1.0-min)/(float)(max - min);
        }
        IDSGEXXDriverLDBG_<<"VAL:"<<val<<"MIN:"<<min<<" MAX:"<<max<<" PERCENT:"<<percent*100;
    } catch  (const GenericException &e){
        // Error handling.
        IDSGEXXDriverLERR_<<  "An exception occurred during GET of Node:"<<node_name;
        IDSGEXXDriverLERR_<< e.GetDescription();
        return -3;
    } catch(...){
        IDSGEXXDriverLERR_<<  "An exception occurre during GET of Node:"<<node_name;
        return -2;
    }
    return 0;
}
/*
static void setNodeInPercentage(const GenApi::CFloatPtr& node,float percent){
    double min,max,inc;
    if(node.get()==NULL){
        IDSGEXXDriverLERR_<<"node not present";
        return;
    }
    min=node->GetMin() ;
    max=node->GetMax();
    inc=node->GetInc();
    IDSGEXXDriverLDBG_<<"MIN:"<<min<<" MAX:"<<max<<" INC:"<<inc;
    double newGainRaw = min+ ((max - min)* percent);
    // Make sure the calculated value is valid
    newGainRaw = Adjust(newGainRaw,  min, max, inc);

    node->SetValue(newGainRaw);
}
*/
class CConfigurationEvent : public CConfigurationEventHandler
{
    ::driver::sensor::camera::IDSGEXXDriver*driver;
public:
    CConfigurationEvent(::driver::sensor::camera::IDSGEXXDriver*drv):driver(drv){};

    void OnAttach( CInstantCamera& /*camera*/)
    {
        IDSGEXXDriverLDBG_ << "OnAttach event";
    }

    void OnAttached( CInstantCamera& camera)
    {
        IDSGEXXDriverLDBG_<< "OnAttached event for device " << camera.GetDeviceInfo().GetModelName() ;
    }

    void OnOpen( CInstantCamera& camera)
    {
        IDSGEXXDriverLDBG_<< "OnOpen event for device " << camera.GetDeviceInfo().GetModelName() ;
    }

    void OnOpened( CInstantCamera& camera)
    {
        driver->propsToCamera(camera,NULL);
    }

    void OnGrabStart( CInstantCamera& camera)
    {
        IDSGEXXDriverLDBG_<< "OnGrabStart event for device " << camera.GetDeviceInfo().GetModelName() ;
    }

    void OnGrabStarted( CInstantCamera& camera)
    {
        IDSGEXXDriverLDBG_<< "OnGrabStarted event for device " << camera.GetDeviceInfo().GetModelName() ;
    }

    void OnGrabStop( CInstantCamera& camera)
    {
        IDSGEXXDriverLDBG_<< "OnGrabStop event for device " << camera.GetDeviceInfo().GetModelName() ;
    }

    void OnGrabStopped( CInstantCamera& camera)
    {
        IDSGEXXDriverLDBG_<< "OnGrabStopped event for device " << camera.GetDeviceInfo().GetModelName() ;
    }

    void OnClose( CInstantCamera& camera)
    {
        IDSGEXXDriverLDBG_<< "OnClose event for device " << camera.GetDeviceInfo().GetModelName() ;
    }

    void OnClosed( CInstantCamera& camera)
    {
        IDSGEXXDriverLDBG_<< "OnClosed event for device " << camera.GetDeviceInfo().GetModelName() ;
    }

    void OnDestroy( CInstantCamera& camera)
    {
        IDSGEXXDriverLDBG_<< "OnDestroy event for device " << camera.GetDeviceInfo().GetModelName() ;
    }

    void OnDestroyed( CInstantCamera& /*camera*/)
    {
        IDSGEXXDriverLDBG_<< "OnDestroyed event" ;
    }

    void OnDetach( CInstantCamera& camera)
    {
        IDSGEXXDriverLDBG_<< "OnDetach event for device " << camera.GetDeviceInfo().GetModelName() ;
    }

    void OnDetached( CInstantCamera& camera)
    {
        IDSGEXXDriverLDBG_<< "OnDetached event for device " << camera.GetDeviceInfo().GetModelName() ;
    }

    void OnGrabError( CInstantCamera& camera, const String_t errorMessage)
    {
        IDSGEXXDriverLDBG_<< "OnGrabError event for device " << camera.GetDeviceInfo().GetModelName() ;
        IDSGEXXDriverLDBG_<< "Error Message: " << errorMessage  ;
    }

    void OnCameraDeviceRemoved( CInstantCamera& camera)
    {
        IDSGEXXDriverLDBG_<< "OnCameraDeviceRemoved event for device " << camera.GetDeviceInfo().GetModelName() ;
    }
};

class CCameraEvent : public CCameraEventHandler
{
public:
    virtual void OnCameraEvent( CInstantCamera& camera, intptr_t userProvidedId, GenApi::INode* pNode)
    {
        IDSGEXXDriverLDBG_<<  "OnCameraEvent event for device " << camera.GetDeviceInfo().GetModelName() ;
        IDSGEXXDriverLDBG_<<  "User provided ID: " << userProvidedId ;
        IDSGEXXDriverLDBG_<<  "Event data node name: " << pNode->GetName() ;
        GenApi::CValuePtr ptrValue( pNode );
        if ( ptrValue.IsValid() )
        {
            IDSGEXXDriverLDBG_<<  "Event node data: " << ptrValue->ToString();
        }
    }
};
}}}
//GET_PLUGIN_CLASS_DEFINITION
//we need to define the driver with alias version and a class that implement it
int IDSGEXXDriver::initializeCamera(const chaos::common::data::CDataWrapper& json) {
    IPylonDevice* pdev=NULL;
    int found=0;
    if(camera){
        IDSGEXXDriverLDBG_<<  "Deleting camera before";

        camera->Close();
        delete camera;
        camera =NULL;
    }
    if(camera==NULL){
        PylonInitialize();
        IDSGEXXDriverLDBG_<<  "Pylon driver initialized";
        // Create an instant camera object for the camera device found first.
        IDSGEXXDriverLDBG_<<"getting  camera informations ..";
        if(json.hasKey("serial")&&json.isStringValue("serial")){
            std::string serial=json.getCStringValue("serial");
            CTlFactory& tlFactory = CTlFactory::GetInstance();
           DeviceInfoList_t devices;
                   // Get all attached devices and exit application if no device is found.
           if ( tlFactory.EnumerateDevices(devices) == 0 ){
               IDSGEXXDriverLERR_<<" No camera present!!!";
               return -1;
            }
           CInstantCameraArray cameras( devices.size());
           IDSGEXXDriverLDBG_<<  "Found "<< cameras.GetSize()<<" cameras, looking for serial:"<<serial;

           for ( size_t i = 0; (i < cameras.GetSize())&& (found==0); ++i){
                pdev=tlFactory.CreateDevice( devices[ i ]);

                cameras[ i ].Attach(pdev);


                IDSGEXXDriverLDBG_<<i<<"] Model:"<< cameras[ i ].GetDeviceInfo().GetModelName();
                IDSGEXXDriverLDBG_ <<i<<"] Friendly Name: " << cameras[ i ].GetDeviceInfo().GetFriendlyName();
                IDSGEXXDriverLDBG_<<i<< "] Full Name    : " << cameras[ i ].GetDeviceInfo().GetFullName();

                IDSGEXXDriverLDBG_ <<i<<"] SerialNumber : " << cameras[ i ].GetDeviceInfo().GetSerialNumber();

                if(serial ==cameras[ i ].GetDeviceInfo().GetSerialNumber().c_str()){
                   IDSGEXXDriverLDBG_<<" FOUND "<<cameras[ i ].GetDeviceInfo().GetSerialNumber();
                   cameras[ i ].DetachDevice();

                   camera=new CInstantCamera(pdev);
                   return 0;
                } else {
                    IDSGEXXDriverLDBG_<<" removing "<<cameras[ i ].GetDeviceInfo().GetSerialNumber();
                    cameras[ i ].DetachDevice();
                    tlFactory.DestroyDevice(pdev);
                  // delete pdev;

                }
           }

        } else {
            IDSGEXXDriverLDBG_<<"No \"serial\" specified getting first camera..";
            pdev=CTlFactory::GetInstance().CreateFirstDevice();
            camera = new CInstantCamera(pdev);
            IDSGEXXDriverLDBG_<<" Model:"<< camera->GetDeviceInfo().GetModelName();
            IDSGEXXDriverLDBG_ << "Friendly Name: " << camera->GetDeviceInfo().GetFriendlyName();
            IDSGEXXDriverLDBG_<< "Full Name    : " <<  camera->GetDeviceInfo().GetFullName();
            IDSGEXXDriverLDBG_ << "SerialNumber : " << camera->GetDeviceInfo().GetSerialNumber() ;
            return 0;

        }
    }
    return -1;
}
IDSGEXXDriver::IDSGEXXDriver():camera(NULL),shots(0),framebuf(NULL),fn(NULL),props(NULL),tmode(CAMERA_TRIGGER_CONTINOUS),gstrategy(CAMERA_LATEST_ONLY){

    IDSGEXXDriverLDBG_<<  "Created Driver";
    props=new chaos::common::data::CDataWrapper();


}
//default descrutcor
IDSGEXXDriver::~IDSGEXXDriver() {
    if(camera){
        IDSGEXXDriverLDBG_<<"deleting camera";

        delete camera;
        camera =NULL;
    }
    if(props){
        delete props;
    }
}
#define GETINTVALUE(x,y){\
    int32_t val;\
    IDSGEXXDriverLDBG_<<"GETTING PROP \""<< # x <<"\" alias:"<<y;\
    if(getNode(# x,camera,val)==0){\
    p->addInt32Value(y,(int32_t)val);}}

#define GETINTPERCVALUE(x,y){ \
    IDSGEXXDriverLDBG_<<"GETTING PROP \""<< # x <<"\" alias:"<<y;\
    float per;\
    if(getNodeInPercentage(# x,camera,per)==0){\
    p->addInt32Value(y,(int32_t)per);}}

int IDSGEXXDriver::cameraToProps(Pylon::CInstantCamera& camera,chaos::common::data::CDataWrapper*p){
    using namespace GenApi;

    if(p == NULL){
        p = props;
    }
    GETINTVALUE(Width,"WIDTH");
    GETINTVALUE(Height,"HEIGHT");
    GETINTVALUE(OffsetX,"OFFSETX");
    GETINTVALUE(OffsetY,"OFFSETY");
    GETINTPERCVALUE(GainRaw,"GAIN");
    GETINTPERCVALUE(ExposureTimeRaw,"SHUTTER");
    GETINTPERCVALUE(SharpnessEnhancementRaw,"SHARPNESS");
    GETINTPERCVALUE(BslBrightnessRaw,"BRIGHTNESS");
    GETINTPERCVALUE(BslContrastRaw,"CONTRAST");
}

int IDSGEXXDriver::propsToCamera(CInstantCamera& camera,chaos::common::data::CDataWrapper*p){
    // Get the camera control object.
    int ret=0;
    INodeMap &control = camera.GetNodeMap();
    if(p == NULL){
        p = props;
    }
    IDSGEXXDriverLDBG_<<"setting props: " << p->getCompliantJSONString() ;

    // Get the parameters for setting the image area of interest (Image AOI).

    // Maximize the Image AOI.
    //   chaos::common::data::CDataWrapper*p=driver->props;

    if (p->hasKey("OFFSETX")){
        if(setNode("OffsetX",camera,(int64_t)p->getInt32Value("OFFSETX"))==0){
            IDSGEXXDriverLDBG_<< "setting OFFSETX " << p->getInt32Value("OFFSETX");
        }else {
            ret++;
            IDSGEXXDriverLERR_<<"Setting OFFSETX FAILED";

        }
    }
    if (p->hasKey("OFFSETY"))
    {
        if(setNode("OffsetY",camera,(int64_t)p->getInt32Value("OFFSETY"))==0){
            IDSGEXXDriverLDBG_<< "setting OFFSETY " << p->getInt32Value("OFFSETY");

        }else {
            ret++;
            IDSGEXXDriverLERR_<<"Setting OFFSETY FAILED";

        }


    }
    if ( p->hasKey("WIDTH")){
        if(setNode("Width",camera,(int64_t)p->getInt32Value("WIDTH"))==0){
            IDSGEXXDriverLDBG_<< "setting WIDTH " << p->getInt32Value("WIDTH");

        } else {
            ret++;
            IDSGEXXDriverLERR_<<"Setting WIDTH FAILED";

        }


    }
    if ( p->hasKey("HEIGHT")){
        if(setNode("Height",camera,(int64_t)p->getInt32Value("HEIGHT"))==0){
            IDSGEXXDriverLDBG_<< "setting HEIGHT " << p->getInt32Value("HEIGHT");

        }else {
            ret++;
            IDSGEXXDriverLERR_<<"Setting HEIGHT FAILED";

        }

    }
    if (p->hasKey("PIXELFMT")){
        // Set the pixel data format.
        CEnumerationPtr(control.GetNode("PixelFormat"))->FromString(p->getCStringValue("PIXELFMT"));
        IDSGEXXDriverLDBG_<< "setting Pixel Format " <<p->getCStringValue("PIXELFMT");
    }
    if(p->hasKey("GAIN")){
        if(p->getDoubleValue("GAIN")<0){
            if(setNode("GainAuto",camera, (int64_t)Basler_GigECamera::GainAutoEnums::GainAuto_Continuous)!=0){
                ret++;
            }


        } else {
            setNode("GainAuto",camera,(int64_t)Basler_GigECamera::GainAutoEnums::GainAuto_Off);


            double gain=p->getDoubleValue("GAIN")/100.0;
            if(gain>1.0){
                gain=1.0;
            }
            if(setNodeInPercentage("GainRaw",camera,gain)!=0){
                ret++;
                IDSGEXXDriverLERR_<<"Setting GAIN FAILED";

            }


            //                   IDSGEXXDriverLDBG_ << "Gain "<<gain<<"     : " << gain_node->GetValue() << " (Min: " << gain_node->GetMin() << "; Max: " <<  gain_node->GetMax() << "; Inc: " <<gain_node->GetInc() << ")";

        }
    }
    if(p->hasKey("SHUTTER")){

        if((p->getDoubleValue("SHUTTER")<0)){
            if(setNode("ExposureAuto",camera,(int64_t)Basler_GigECamera::ExposureAutoEnums::ExposureAuto_Continuous)!=0){
                ret++;
            }


        } else {
            setNode("ExposureAuto",camera,(int64_t)Basler_GigECamera::ExposureAutoEnums::ExposureAuto_Off);


            double gain=p->getDoubleValue("SHUTTER")/100.0;
            if(gain>1.0){
                gain=1.0;
            }
            if(setNodeInPercentage("ExposureTimeRaw",camera,gain)!=0){
                ret++;
                IDSGEXXDriverLERR_<<"Setting SHUTTER FAILED";

            }


        }
    }
    if(p->hasKey("SHARPNESS")){

        if(p->getDoubleValue("SHARPNESS")<0){
            setNode("DemosaicingMode",camera,(int64_t)Basler_GigECamera::DemosaicingModeEnums::DemosaicingMode_BaslerPGI );

        } else {


            double gain=p->getDoubleValue("SHARPNESS")/100.0;
            if(gain>1.0){
                gain=1.0;
            }
            if(setNodeInPercentage("SharpnessEnhancementRaw",camera,gain)!=0){
                ret++;
                IDSGEXXDriverLERR_<<"Setting SHARPNESS FAILED";

            }

            //                    IDSGEXXDriverLDBG_ << "SHARPNESS "<<gain<<"     : " << sharp_node->GetValue() << " (Min: " << sharp_node->GetMin() << "; Max: " <<  sharp_node->GetMax() << "; Inc: " <<sharp_node->GetInc() << ")";

        }
    }
    if(p->hasKey("BRIGHTNESS")){
        setNode("DemosaicingMode",camera,(int64_t)Basler_GigECamera::DemosaicingModeEnums::DemosaicingMode_BaslerPGI );

        if(p->getDoubleValue("BRIGHTNESS")<0){

        } else {


            double gain=p->getDoubleValue("BRIGHTNESS")/100.0;
            if(gain>1.0){
                gain=1.0;
            }
            if(setNodeInPercentage("BslBrightnessRaw",camera,gain)!=0){
                ret++;
                IDSGEXXDriverLERR_<<"Setting BRIGHTNESS FAILED";

            }

            //         IDSGEXXDriverLDBG_ << "BRIGHTNESS "<<gain<<"     : " << bright_node->GetValue() << " (Min: " << bright_node->GetMin() << "; Max: " <<  bright_node->GetMax() << "; Inc: " <<bright_node->GetInc() << ")";

        }
    }
    if(p->hasKey("CONTRAST")){
        setNode("DemosaicingMode",camera,(int64_t)Basler_GigECamera::DemosaicingModeEnums::DemosaicingMode_BaslerPGI );

        if(p->getDoubleValue("CONTRAST")<0){

        } else {


            double gain=p->getDoubleValue("CONTRAST")/100.0;
            if(gain>1.0){
                gain=1.0;
            }
            if(setNodeInPercentage("BslContrastRaw",camera,gain)!=0){
                ret++;
                  IDSGEXXDriverLERR_<<"Setting CONTRAST FAILED";
            }

            //                  IDSGEXXDriverLDBG_ << "CONTRAST "<<gain<<"     : " << contrast_node->GetValue() << " (Min: " << contrast_node->GetMin() << "; Max: " <<  contrast_node->GetMax() << "; Inc: " <<contrast_node->GetInc() << ")";

        }
    }
    //         p->addInt32Value("WIDTH", (int32_t)width->GetValue());
    //       p->addInt32Value("HEIGHT", (int32_t)height->GetValue());

    return ret;
}

int IDSGEXXDriver::cameraInit(void *buffer,uint32_t sizeb){
    IDSGEXXDriverLDBG_<<"Initialization";
    // simple initialization
    if(camera==NULL){
        IDSGEXXDriverLERR_<<"no camera available";
        return -1;
    }
    try {
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

        IDSGEXXDriverLDBG_<<"trigger configuration handler installed";

        // For demonstration purposes only, add sample configuration event handlers to print out information
        // about camera use and image grabbing.
        camera->RegisterConfiguration( new CConfigurationEvent(this), RegistrationMode_Append, Cleanup_Delete);
        IDSGEXXDriverLDBG_<<"event  handler installed";

        //camera->RegisterImageEventHandler( new CCameraEventPrinter, RegistrationMode_Append, Cleanup_Delete);
        // Print the model name of the camera.
        IDSGEXXDriverLDBG_<< "Using device " << camera->GetDeviceInfo().GetModelName();

        // The MaxNumBuffer parameter can be used to control the count of buffers
        // allocated for grabbing. The default value of this parameter is 10.
        camera->MaxNumBuffer = 15;
        IDSGEXXDriverLDBG_<<"Open camera";

        // Open the camera.
        camera->Open();

        setNode("DemosaicingMode",*camera,(int64_t)Basler_GigECamera::DemosaicingModeEnums::DemosaicingMode_BaslerPGI );



    } catch  (const GenericException &e){
        // Error handling.
        IDSGEXXDriverLERR_<<  "An exception occurred.";
        IDSGEXXDriverLERR_<< e.GetDescription();
        return -2;
    } catch(...){
        IDSGEXXDriverLERR_<<  "An exception occurred.";
        return -3;
    }


    return 0;
}

int IDSGEXXDriver::cameraDeinit(){
    IDSGEXXDriverLDBG_<<"deinit";
    if(camera){
        camera->Close();

    }
    return 0;
}


int IDSGEXXDriver::startGrab(uint32_t _shots,void*_framebuf,cameraGrabCallBack _fn){
    IDSGEXXDriverLDBG_<<"Start Grabbing";
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

int IDSGEXXDriver::waitGrab(uint32_t timeout_ms){
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
            IDSGEXXDriverLDBG_<< "Skipped " << ptrGrabResult->GetNumberOfSkippedImages() << " image.";
        }
        if (ptrGrabResult->GrabSucceeded()){
            // Access the image data.

            const uint8_t *pImageBuffer = (uint8_t *) ptrGrabResult->GetBuffer();
            //      cout << "Gray value of first pixel: " << (uint32_t) pImageBuffer[0] << endl << endl;


            IDSGEXXDriverLDBG_<<"SizeX: " << ptrGrabResult->GetWidth() ;
            IDSGEXXDriverLDBG_<<"SizeY: " << ptrGrabResult->GetHeight();
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
	    IDSGEXXDriverLDBG_<<"Image Raw Size: " << size_ret ;
            memcpy(framebuf,pImageBuffer,size_ret);


            return size_ret;

        }
        else
        {
            IDSGEXXDriverLERR_<< "Error: " << ptrGrabResult->GetErrorCode() << " " << ptrGrabResult->GetErrorDescription();
        }

        nBuffersInQueue++;
    }

    IDSGEXXDriverLDBG_<<"Retrieved " << nBuffersInQueue << " grab results from output queue.";

    return 0;

}
int IDSGEXXDriver::stopGrab(){
    camera->StopGrabbing();
}

int  IDSGEXXDriver::setImageProperties(uint32_t width,uint32_t height,uint32_t opencvImageType){
    props->addInt32Value("WIDTH",width);
    props->addInt32Value("HEIGHT",height);
    if(camera){
        return propsToCamera(*camera,props);
    }
    return -1;
}

int  IDSGEXXDriver::getImageProperties(uint32_t& width,uint32_t& height,uint32_t& opencvImageType){
    ChaosUniquePtr<chaos::common::data::CDataWrapper> cw(new chaos::common::data::CDataWrapper());
    int ret=-1;
    cameraToProps(*camera,cw.get());
    if(cw->hasKey("WIDTH")){
        width=cw->getInt32Value("WIDTH");
        ret=0;
    }

    if(cw->hasKey("HEIGHT")){
        height=cw->getInt32Value("HEIGHT");
        ret++;
    }

    return (ret>0)?0:ret;
}



int  IDSGEXXDriver::setCameraProperty(const std::string& propname,uint32_t val){
    ChaosUniquePtr<chaos::common::data::CDataWrapper> cw(new chaos::common::data::CDataWrapper());
    IDSGEXXDriverLDBG_<<"Setting \"" << propname<<"\"="<<(int32_t)val ;

    cw->addInt32Value(propname,(int32_t)val);

    return propsToCamera(*camera,cw.get());

}
int  IDSGEXXDriver::setCameraProperty(const std::string& propname,double val){
    ChaosUniquePtr<chaos::common::data::CDataWrapper> cw(new chaos::common::data::CDataWrapper());
    IDSGEXXDriverLDBG_<<"Setting \"" << propname<<"\"="<<val ;

    cw->addDoubleValue(propname,val);

    return propsToCamera(*camera,cw.get());

}

int  IDSGEXXDriver::getCameraProperty(const std::string& propname,uint32_t& val){
    ChaosUniquePtr<chaos::common::data::CDataWrapper> cw(new chaos::common::data::CDataWrapper());

    cameraToProps(*camera,cw.get());

    if(cw->hasKey(propname)){
        val = props->getInt32Value(propname);
        return 0;
    }
    return -1;
}

int  IDSGEXXDriver::getCameraProperty(const std::string& propname,double& val){
    ChaosUniquePtr<chaos::common::data::CDataWrapper> cw(new chaos::common::data::CDataWrapper());

    cameraToProps(*camera,cw.get());

    if(cw->hasKey(propname)){
        val = props->getDoubleValue(propname);
        return 0;
    }
    return -1;
}

int  IDSGEXXDriver::getCameraProperties(std::vector<std::string >& proplist){
    ChaosUniquePtr<chaos::common::data::CDataWrapper> cw(new chaos::common::data::CDataWrapper());
    cameraToProps(*camera,cw.get());

    cw->getAllKey(proplist);
    return (proplist.size()>0)?0:-1;
}
