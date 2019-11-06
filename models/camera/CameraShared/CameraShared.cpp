/*
 *	CameraShared.h
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
#include "CameraShared.h"
#include <stdlib.h>

#include <string>
#include <chaos/common/data/CDataWrapper.h>
#include <chaos/cu_toolkit/driver_manager/driver/AbstractDriverPlugin.h>
#include <math.h>
#include <boost/lexical_cast.hpp>

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int_distribution.hpp>

namespace cu_driver = chaos::cu::driver_manager::driver;
using namespace ::driver::sensor::camera;
using namespace cv;

#define CameraSharedLAPP_		LAPP_ << "[CameraShared] "
#define CameraSharedLDBG_		LDBG_ << "[CameraShared:"<<__PRETTY_FUNCTION__<<"]"
#define CameraSharedLERR_		LERR_ << "[CameraShared:"<<__PRETTY_FUNCTION__<<"]"


//GET_PLUGIN_CLASS_DEFINITION
//we need only to define the driver because we don't are makeing a plugin
OPEN_CU_DRIVER_PLUGIN_CLASS_DEFINITION(CameraShared, 1.0.0,::driver::sensor::camera::CameraShared)
REGISTER_CU_DRIVER_PLUGIN_CLASS_INIT_ATTRIBUTE(::driver::sensor::camera::CameraShared, http_address/dnsname:port)
CLOSE_CU_DRIVER_PLUGIN_CLASS_DEFINITION
OPEN_REGISTER_PLUGIN
REGISTER_PLUGIN(::driver::sensor::camera::CameraShared)
CLOSE_REGISTER_PLUGIN

void CameraShared::driverInit(const char *initParameter) throw(chaos::CException){
    CameraSharedLAPP_ << "Initializing  driver:"<<initParameter;
    props->reset();
    if(initializeCamera(*props)!=0){
        throw chaos::CException(-3,"cannot initialize camera ",__PRETTY_FUNCTION__);

    }
}


void CameraShared::driverInit(const chaos::common::data::CDataWrapper& json) throw(chaos::CException){
    CameraSharedLAPP_ << "Initializing  driver:"<<json.getCompliantJSONString();
    if(initializeCamera(json)!=0){
        throw chaos::CException(-1,"cannot initialize camera "+json.getCompliantJSONString(),__PRETTY_FUNCTION__);
    }

}

void CameraShared::driverDeinit() throw(chaos::CException) {
    CameraSharedLAPP_ << "Deinit driver";
    if(framebuf[0]){
        free(framebuf[0]);
        framebuf[0]=NULL;
    }
    framebuf_size[0]=0;
if(framebuf[1]){
        free(framebuf[1]);
        framebuf[1]=NULL;
    }
    framebuf_size[1]=0;

}

namespace driver{

namespace sensor{
namespace camera{


//GET_PLUGIN_CLASS_DEFINITION
//we need to define the driver with alias version and a class that implement it
int CameraShared::initializeCamera(const chaos::common::data::CDataWrapper& json) {
    int ret=-2;
    CameraSharedLDBG_<<"initialize camera:"<<json.getCompliantJSONString();
    if(json.hasKey("src")){
        if(json.isStringValue("src")){
            std::string name=json.getStringValue("src");
            try{
            shared_mem.reset(new ::common::misc::data::SharedMem(name));
            CameraSharedLDBG_ << "opened sharedMem \""<<shared_mem->getName()<<"\" at @"<<std::hex<<shared_mem->getMem()<<" size:"<<std::dec<<shared_mem->getSize();
           uint8_t* buf;
           size_t siz=shared_mem->readMsg(&buf);
            if(siz && buf){
                try{
                    std::vector<uchar> data = std::vector<uchar>(buf, buf + siz);
                    cv::Mat m=cv::imdecode(data,cv::IMREAD_UNCHANGED);
                    width=m.cols;
                    height=m.rows;
                    CameraSharedLDBG_ << "Found image of "<<width<<"x"<<height;
                } catch(...){
                    std::stringstream ss;
                    ss<< "Invalid image found at:@ "<<std::hex<<buf<<std::dec;
                    throw chaos::CException(-10, ss.str(), __PRETTY_FUNCTION__);
                }
            }
            return 0;
            } catch(boost::interprocess::interprocess_exception &ex){

                throw chaos::CException(-1, "cannot  open shared memory : " + std::string(ex.what()), __PRETTY_FUNCTION__);

        }
    } else {
        throw chaos::CException(-2, "key 'src' should be a string", __PRETTY_FUNCTION__);


    }

} else {
    throw chaos::CException(-2, "expected a 'src' key as string", __PRETTY_FUNCTION__);
}   
    return ret;
}
DEFAULT_CU_DRIVER_PLUGIN_CONSTRUCTOR_WITH_NS(::driver::sensor::camera,CameraShared),shots(0),frames(0),fn(NULL),props(NULL),tmode(CAMERA_TRIGGER_CONTINOUS),gstrategy(CAMERA_LATEST_ONLY),initialized(false),height(CAM_DEFAULT_HEIGTH),width(CAM_DEFAULT_WIDTH),framerate(1.0),offsetx(0),offsety(0){

    CameraSharedLDBG_<<  "Created Driver";
    framebuf[0]=NULL;
    framebuf[1]=NULL;
    framebuf_size[0]=0;
    framebuf_size[1]=0;
    movex=movey=rot=0;
    props=new chaos::common::data::CDataWrapper();
#ifdef CVDEBUG

    cv::namedWindow("test");
#endif
}
//default descrutcor
CameraShared::~CameraShared() {

}

int CameraShared::cameraToProps(chaos::common::data::CDataWrapper*p){

    if(p == NULL){
        p = props;
    }
    CameraSharedLDBG_<< "WIDTH:"<<width<<" HEIGHT:"<<height;
    CameraSharedLDBG_<< "FRAMERATE:"<<framerate;
    p->addDoubleValue("FRAMERATE",framerate);
    p->addInt32Value("WIDTH",width);
    p->addInt32Value("HEIGHT",height);
    p->addInt32Value("TRIGGER_MODE",0);

    p->addInt32Value("OFFSETX",offsetx);
    p->addInt32Value("OFFSETY",offsety);
    return 0;
}

#define PXL2COLOR(col) \
    if(fmt == #col) {\
    color=col;}

int CameraShared::propsToCamera(chaos::common::data::CDataWrapper*p){
    // Get the camera control object.
    int32_t ret=0;

    if(p == NULL){
        p = props;
    }
    CameraSharedLDBG_<<"setting props: " << p->getCompliantJSONString() ;

    // Get the parameters for setting the image area of interest (Image AOI).

    // Maximize the Image AOI.
    //   chaos::common::data::CDataWrapper*p=driver->props;

    if (p->hasKey("OFFSETX")){
        offsetx=p->getInt32Value("OFFSETX");
        CameraSharedLDBG_<< "setting OFFSETX " << offsetx;

    }
    if (p->hasKey("OFFSETY")){
        offsety= p->getInt32Value("OFFSETY");
        CameraSharedLDBG_<< "setting OFFSET " << offsety;

    }
    if ( p->hasKey("WIDTH")){
        CameraSharedLDBG_<< "setting WIDTH " << p->getInt32Value("WIDTH");
        width=p->getInt32Value("WIDTH");
    }

    if ( p->hasKey("HEIGHT")){
        CameraSharedLDBG_<< "setting HEIGHT " << p->getInt32Value("HEIGHT");
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
        CameraSharedLDBG_<< "FRAME RATE:"<<framerate;
    }


    if(p->hasKey("SHARPNESS")){
    }
    if(p->hasKey("BRIGHTNESS")){
    }
    if(p->hasKey("CONTRAST")){
    }


    return ret;
}

int CameraShared::cameraInit(void *buffer,uint32_t sizeb){
    CameraSharedLDBG_<<"Initialization";
    // simple initialization
    frames=0;




    // For demonstration purposes only, add sample configuration event handlers to print out information
    // about camera use and image grabbing.
    return 0;
}

int CameraShared::cameraDeinit(){
    CameraSharedLDBG_<<"deinit";
    frames=0;

    return 0;
}

#define GETDBLPARAM(w,name) \
    if(w->hasKey(#name)){\
    name = w->getDoubleValue(#name);\
    CameraSharedLDBG_<< "shape DBL param '"<<#name<<" = "<<name;}
#define GETINTPARAM(w,name) \
    if(w->hasKey(#name)){\
    name = w->getInt32Value(#name);\
    CameraSharedLDBG_<< "shape INT param '"<<#name<<" = "<<name;}
    
int CameraShared::startGrab(uint32_t _shots,void*_framebuf,cameraGrabCallBack _fn){
    int ret=-1;
    shots=_shots;
    //framebuf=_framebuf;
  

       last_acquisition_ts=chaos::common::utility::TimingUtil::getTimeStampInMicroseconds();
    CameraSharedLDBG_<<"Start Grabbing at:"<<framerate <<" frame/s";
    return ret;
}
#define RND_DIST(var) \
    tmp_##var=var;\
    if(err_##var>0){\
    boost::random::uniform_int_distribution<> dist_##var(-err_##var,err_##var);\
    double err=dist_##var(gen);\
    tmp_##var+= err;}

int CameraShared::waitGrab(const char**buf,uint32_t timeout_ms){
    int32_t ret=-1;
    if(shared_mem->wait(timeout_ms)==0){
        uint8_t* bufs;
        size_t siz=shared_mem->readMsg(&bufs);
        if(siz&&bufs){
            std::vector<uchar> data = std::vector<uchar>(bufs, bufs + siz);

            cv::Mat img=cv::imdecode(data,cv::IMREAD_UNCHANGED);
            width=img.cols;
            height=img.rows;
            int size = img.total() * img.elemSize();
            ret= size;
            if(framebuf_size[frames&1]<size){
                free(framebuf[frames&1]);
                framebuf[frames&1]=malloc(size);
                framebuf_size[frames&1]=size;
            }
            
            std::memcpy(framebuf[frames&1],img.data,size );
            if(buf){
                if(frames>0){
                    *buf=(char*)framebuf[!(frames&1)];
            } else {
                    *buf=(char*)framebuf[0];
            }
        } else {
            CameraSharedLERR_<<"BAD BUFFER GIVEN "<<shape_type<<"("<<width<<"X"<<height<<")"<<frames<<" center "<<tmp_centerx<<","<<tmp_centery<<" sizex:"<<tmp_sizex<<" sizey:"<<tmp_sizey<<" color:"<<colr<<"R,"<<colg<<"G,"<<colb<<" size byte:"<<size;

        }
        }


    }
     


    return ret;

}

int CameraShared::waitGrab(uint32_t timeout_ms){
        CameraSharedLERR_<<"NOT IMPLEMENTED";

    return 0;
    //return waitGrab((const char**)&framebuf,timeout_ms);
}
int CameraShared::stopGrab(){
    //camera->StopGrabbing();
    return 0;
}

int  CameraShared::setImageProperties(int32_t width,int32_t height,int32_t opencvImageType){
    props->addInt32Value("WIDTH",width);
    props->addInt32Value("HEIGHT",height);
    props->addDoubleValue("FRAMERATE",framerate);

    return propsToCamera(props);
}

int  CameraShared::getImageProperties(int32_t& width,int32_t& height,int32_t& opencvImageType){
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



int  CameraShared::setCameraProperty(const std::string& propname,int32_t val){
    ChaosUniquePtr<chaos::common::data::CDataWrapper> cw(new chaos::common::data::CDataWrapper());
    CameraSharedLDBG_<<"Setting \"" << propname<<"\"="<<(int32_t)val ;

    cw->addInt32Value(propname,(int32_t)val);

    return propsToCamera(cw.get());

}
int  CameraShared::setCameraProperty(const std::string& propname,double val){
    ChaosUniquePtr<chaos::common::data::CDataWrapper> cw(new chaos::common::data::CDataWrapper());
    CameraSharedLDBG_<<"Setting \"" << propname<<"\"="<<val ;

    cw->addDoubleValue(propname,val);

    return propsToCamera(cw.get());

}

int  CameraShared::getCameraProperty(const std::string& propname,int32_t& val){
    ChaosUniquePtr<chaos::common::data::CDataWrapper> cw(new chaos::common::data::CDataWrapper());

    cameraToProps(cw.get());

    if(cw->hasKey(propname)){
        val = cw->getInt32Value(propname);
        return 0;
    }
    return -1;
}

int  CameraShared::getCameraProperty(const std::string& propname,double& val){
    ChaosUniquePtr<chaos::common::data::CDataWrapper> cw(new chaos::common::data::CDataWrapper());

    cameraToProps(cw.get());
    if(cw->hasKey(propname)){
        val = cw->getDoubleValue(propname);
        return 0;
    }
    return -1;
}

int  CameraShared::getCameraProperties(chaos::common::data::CDataWrapper& proplist){

    return cameraToProps(&proplist);


}
}}}
