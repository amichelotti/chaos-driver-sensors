/*
 *	ShapeSim.h
 *	!CHAOS
 *	Created by Andrea Michelotti.
 *
 *    	Copyright 2018 INFN, National Institute of Nuclear Physics
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
#include "ShapeSim.h"
#include <stdlib.h>
#include <string>
#include <chaos/common/data/CDataWrapper.h>
#include <chaos/cu_toolkit/driver_manager/driver/AbstractDriverPlugin.h>
#include <math.h>
#include <boost/lexical_cast.hpp>
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
namespace cu_driver = chaos::cu::driver_manager::driver;
using namespace ::driver::sensor::camera;
using namespace cv;

#define ShapeSimLAPP_		LAPP_ << "[ShapeSim] "
#define ShapeSimLDBG_		LDBG_ << "[ShapeSim:"<<__PRETTY_FUNCTION__<<"]"
#define ShapeSimLERR_		LERR_ << "[ShapeSim:"<<__PRETTY_FUNCTION__<<"]"


//GET_PLUGIN_CLASS_DEFINITION
//we need only to define the driver because we don't are makeing a plugin
OPEN_CU_DRIVER_PLUGIN_CLASS_DEFINITION(ShapeSim, 1.0.0,::driver::sensor::camera::ShapeSim)
REGISTER_CU_DRIVER_PLUGIN_CLASS_INIT_ATTRIBUTE(::driver::sensor::camera::ShapeSim, http_address/dnsname:port)
CLOSE_CU_DRIVER_PLUGIN_CLASS_DEFINITION
OPEN_REGISTER_PLUGIN
REGISTER_PLUGIN(::driver::sensor::camera::ShapeSim)
CLOSE_REGISTER_PLUGIN

void ShapeSim::driverInit(const char *initParameter) throw(chaos::CException){
    ShapeSimLAPP_ << "Initializing  driver:"<<initParameter;
    props->reset();
    if(initializeCamera(*props)!=0){
        throw chaos::CException(-3,"cannot initialize camera ",__PRETTY_FUNCTION__);

    }
}


void ShapeSim::driverInit(const chaos::common::data::CDataWrapper& json) throw(chaos::CException){
    ShapeSimLAPP_ << "Initializing  driver:"<<json.getCompliantJSONString();
    props->reset();
    props->appendAllElement((chaos::common::data::CDataWrapper&)json);
    if(initializeCamera(*props)!=0){
        throw chaos::CException(-1,"cannot initialize camera "+json.getCompliantJSONString(),__PRETTY_FUNCTION__);
    }

}

void ShapeSim::driverDeinit() throw(chaos::CException) {
    ShapeSimLAPP_ << "Deinit driver";

}

namespace driver{

namespace sensor{
namespace camera{


//GET_PLUGIN_CLASS_DEFINITION
//we need to define the driver with alias version and a class that implement it
int ShapeSim::initializeCamera(const chaos::common::data::CDataWrapper& json) {
    int ret=-2;
    if(json.hasKey("SHAPE")&& json.isCDataWrapperValue("SHAPE")){
        shape_params=json.getCSDataValue("SHAPE");
        if(shape_params!=NULL){
            if(shape_params->hasKey("type")){
                ret=0;
                initialized=true;
            } else {
                ShapeSimLERR_<<  "Missing shape 'type'";

            }

        } else {
            ShapeSimLERR_<<  "Error retriving 'SHAPE'";

        }

    } else {
        ShapeSimLERR_<<  "Missing 'SHAPE'";

    }


    return ret;
}
  ShapeSim::ShapeSim():shots(0),framebuf(NULL),fn(NULL),props(NULL),tmode(CAMERA_TRIGGER_CONTINOUS),gstrategy(CAMERA_LATEST_ONLY),initialized(false),height(CAM_DEFAULT_HEIGTH),width(CAM_DEFAULT_WIDTH),framerate(1),offsetx(0),offsety(0),shape_params(NULL){

    ShapeSimLDBG_<<  "Created Driver";
    props=new chaos::common::data::CDataWrapper();


}
//default descrutcor
ShapeSim::~ShapeSim() {

}

int ShapeSim::cameraToProps(chaos::common::data::CDataWrapper*p){

    if(p == NULL){
        p = props;
    }
    ShapeSimLDBG_<< "WIDTH:"<<width<<" HEIGHT:"<<height;
    ShapeSimLDBG_<< "FRAMERATE:"<<framerate;
    p->addDoubleValue("FRAMERATE",framerate);
    p->addInt32Value("WIDTH",width);
    p->addInt32Value("HEIGHT",height);

    p->addInt32Value("OFFSETX",offsetx);
    p->addInt32Value("OFFSETY",offsety);
    return 0;
}

#define PXL2COLOR(col) \
    if(fmt == #col) {\
    color=col;}

int ShapeSim::propsToCamera(chaos::common::data::CDataWrapper*p){
    // Get the camera control object.
    int32_t ret=0;

    if(p == NULL){
        p = props;
    }
    ShapeSimLDBG_<<"setting props: " << p->getCompliantJSONString() ;

    // Get the parameters for setting the image area of interest (Image AOI).

    // Maximize the Image AOI.
    //   chaos::common::data::CDataWrapper*p=driver->props;

    if (p->hasKey("OFFSETX")){
        offsetx=-p->getInt32Value("OFFSETX");
        ShapeSimLDBG_<< "setting OFFSETX " << offsetx;

    }
    if (p->hasKey("OFFSETY")){
        offsety= p->getInt32Value("OFFSETY");
        ShapeSimLDBG_<< "setting OFFSET " << offsety;

    }
    if ( p->hasKey("WIDTH")){
        ShapeSimLDBG_<< "setting WIDTH " << p->getInt32Value("WIDTH");
        width=p->getInt32Value("WIDTH");
    }

    if ( p->hasKey("HEIGHT")){
        ShapeSimLDBG_<< "setting HEIGHT " << p->getInt32Value("HEIGHT");
        height=p->getInt32Value("HEIGHT");
    }
    if (p->hasKey("PIXELFMT")){

    }

    if(p->hasKey("GAIN")){
    }

    if(p->hasKey("SHUTTER")){
    }
    if(p->hasKey("ZOOM")){

    }
    // Pixel Clock

    if(p->hasKey("FRAMERATE")){
        framerate=p->getDoubleValue("FRAMERATE");
        ShapeSimLDBG_<< "FRAME RATE:"<<framerate;
    }


    if(p->hasKey("SHARPNESS")){
    }
    if(p->hasKey("BRIGHTNESS")){
    }
    if(p->hasKey("CONTRAST")){
    }


    return ret;
}

int ShapeSim::cameraInit(void *buffer,uint32_t sizeb){
    ShapeSimLDBG_<<"Initialization";
    // simple initialization




    // For demonstration purposes only, add sample configuration event handlers to print out information
    // about camera use and image grabbing.
    return 0;
}

int ShapeSim::cameraDeinit(){
    ShapeSimLDBG_<<"deinit";


    return 0;
}


int ShapeSim::startGrab(uint32_t _shots,void*_framebuf,cameraGrabCallBack _fn){
    ShapeSimLDBG_<<"Start Grabbing";
    int ret=-1;
    shots=_shots;
    framebuf=_framebuf;
    fn=_fn;
     err_centerx=0;
     err_centery=0;
     err_sizex=0;
     err_sizey=0;

     err_rotangle=0;
     err_extangle=0;
    // fetch parameters
    if(shape_params->hasKey("type")){
        shape_type=shape_params->getStringValue("type");
        ret =0;
        centerx=width/2;
        centery=height/2;
        if(shape_params->hasKey("centerx")){
            centerx=shape_params->getInt32Value("centerx");
        }
        if(shape_params->hasKey("centery")){
            centery=shape_params->getInt32Value("centery");
        }
        if(shape_params->hasKey("err_centerx")){
            err_centerx=shape_params->getDoubleValue("err_centerx");
        }
        if(shape_params->hasKey("err_centery")){
            err_centery=shape_params->getDoubleValue("err_centery");
        }
        if(shape_params->hasKey("err_sizex")){
            err_sizex=shape_params->getDoubleValue("err_sizex");
        }
        if(shape_params->hasKey("err_sizey")){
            err_sizey=shape_params->getDoubleValue("err_sizey");
        }
        if(shape_params->hasKey("err_rotangle")){
            err_rotangle=shape_params->getDoubleValue("err_rotangle");
        }
        if(shape_params->hasKey("err_extangle")){
            err_extangle=shape_params->getDoubleValue("err_extangle");
        }
        sizex=centerx/2;
        sizey=centery/2;
        if(shape_params->hasKey("sizex")){
            sizex=shape_params->getInt32Value("sizex");
        }
        if(shape_params->hasKey("sizey")){
            sizey=shape_params->getInt32Value("sizey");
        }
        rotangle=0;
        extangle=0;
        if(shape_params->hasKey("rotangle")){
            rotangle=shape_params->getDoubleValue("rotangle");
        }
        if(shape_params->hasKey("extangle")){
            extangle=shape_params->getDoubleValue("extangle");
        }
        colr=255;
        colg=0;
        colb=0;
        tickness=2;
        linetype=8; //connected
        if(shape_params->hasKey("colr")){
            colr=shape_params->getInt32Value("colr");
        }
        if(shape_params->hasKey("colg")){
            colg=shape_params->getInt32Value("colg");
        }
        if(shape_params->hasKey("colb")){
            colb=shape_params->getInt32Value("colb");
        }
    }
    return ret;
}

int ShapeSim::waitGrab(uint32_t timeout_ms){
    int32_t ret=-1;
    size_t size_ret;

    if(shape_type == "ellipse"){
        Mat img(width, height, CV_8UC3, Scalar::all(0));
        // get parameters

        ellipse( img,
           Point( centerx+err_centerx, centery+err_centery),
           Size( sizex + err_sizex, sizey+ err_sizey ),
           rotangle + err_rotangle,
           0,
           360,
           Scalar( colr, colg, colb ),
           tickness,
           linetype );

        int size = img.total() * img.elemSize();
        std::memcpy(framebuf,img.data,size );
        ret=0;
    }

/*    if((ret=camera.captureImage(timeout_ms,(char*)framebuf,&size_ret))==0){
        ShapeSimLDBG_<<"Retrieved Image "<<camera.getWidth()<<"x"<<camera.getHeight()<<" raw size:"<<size_ret;
        ret= size_ret;
    } else{
        ShapeSimLERR_<<"No Image..";
    }
*/


    return ret;

}
int ShapeSim::stopGrab(){
    //camera->StopGrabbing();
}

int  ShapeSim::setImageProperties(int32_t width,int32_t height,int32_t opencvImageType){
    props->addInt32Value("WIDTH",width);
    props->addInt32Value("HEIGHT",height);
    props->addDoubleValue("FRAMERATE",framerate);

    return propsToCamera(props);
}

int  ShapeSim::getImageProperties(int32_t& width,int32_t& height,int32_t& opencvImageType){
    ChaosUniquePtr<chaos::common::data::CDataWrapper> cw(new chaos::common::data::CDataWrapper());
    int ret=-1;
    cameraToProps(cw.get());
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



int  ShapeSim::setCameraProperty(const std::string& propname,int32_t val){
    ChaosUniquePtr<chaos::common::data::CDataWrapper> cw(new chaos::common::data::CDataWrapper());
    ShapeSimLDBG_<<"Setting \"" << propname<<"\"="<<(int32_t)val ;

    cw->addInt32Value(propname,(int32_t)val);

    return propsToCamera(cw.get());

}
int  ShapeSim::setCameraProperty(const std::string& propname,double val){
    ChaosUniquePtr<chaos::common::data::CDataWrapper> cw(new chaos::common::data::CDataWrapper());
    ShapeSimLDBG_<<"Setting \"" << propname<<"\"="<<val ;

    cw->addDoubleValue(propname,val);

    return propsToCamera(cw.get());

}

int  ShapeSim::getCameraProperty(const std::string& propname,int32_t& val){
    ChaosUniquePtr<chaos::common::data::CDataWrapper> cw(new chaos::common::data::CDataWrapper());

    cameraToProps(cw.get());

    if(cw->hasKey(propname)){
        val = props->getInt32Value(propname);
        return 0;
    }
    return -1;
}

int  ShapeSim::getCameraProperty(const std::string& propname,double& val){
    ChaosUniquePtr<chaos::common::data::CDataWrapper> cw(new chaos::common::data::CDataWrapper());

    cameraToProps(cw.get());
    if(cw->hasKey(propname)){
        val = props->getDoubleValue(propname);
        return 0;
    }
    return -1;
}

int  ShapeSim::getCameraProperties(chaos::common::data::CDataWrapper& proplist){

    return cameraToProps(&proplist);


}
}}}
