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

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int_distribution.hpp>

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
    if(initializeCamera(json)!=0){
        throw chaos::CException(-1,"cannot initialize camera "+json.getCompliantJSONString(),__PRETTY_FUNCTION__);
    }

}

void ShapeSim::driverDeinit() throw(chaos::CException) {
    ShapeSimLAPP_ << "Deinit driver";
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
int ShapeSim::initializeCamera(const chaos::common::data::CDataWrapper& json) {
    int ret=-2;
    ShapeSimLDBG_<<"intialize camera:"<<json.getCompliantJSONString();
    if(json.hasKey("SHAPE")){
        if(json.isCDataWrapperValue("SHAPE")){
            shape_params=json.getCSDataValue("SHAPE");
            if(shape_params!=NULL){
                if(shape_params->hasKey("type")){
                    ret=0;
                    initialized=true;
                } else {
                    ShapeSimLERR_<<  "Missing shape 'type'"<<json.getCompliantJSONString();

                }

            } else {
                ShapeSimLERR_<<  "  retriving 'SHAPE'"<<json.getCompliantJSONString();

            }

        } else {
            ShapeSimLERR_<<  "Missing 'SHAPE' parameters:"<<json.getCompliantJSONString();

        }
    } else {
        ShapeSimLERR_<<  "Missing 'SHAPE' key:"<<json.getCompliantJSONString();
    }


    return ret;
}
DEFAULT_CU_DRIVER_PLUGIN_CONSTRUCTOR_WITH_NS(::driver::sensor::camera,ShapeSim),shots(0),frames(0),fn(NULL),props(NULL),tmode(CAMERA_TRIGGER_CONTINOUS),gstrategy(CAMERA_LATEST_ONLY),initialized(false),height(CAM_DEFAULT_HEIGTH),width(CAM_DEFAULT_WIDTH),framerate(1.0),offsetx(0),offsety(0){

    ShapeSimLDBG_<<  "Created Driver";
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
    p->addInt32Value("TRIGGER_MODE",0);

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
    frames=0;




    // For demonstration purposes only, add sample configuration event handlers to print out information
    // about camera use and image grabbing.
    return 0;
}

int ShapeSim::cameraDeinit(){
    ShapeSimLDBG_<<"deinit";
    frames=0;

    return 0;
}

#define GETDBLPARAM(w,name) \
    if(w->hasKey(#name)){\
    name = w->getDoubleValue(#name);\
    ShapeSimLDBG_<< "shape DBL param '"<<#name<<" = "<<name;}
#define GETINTPARAM(w,name) \
    if(w->hasKey(#name)){\
    name = w->getInt32Value(#name);\
    ShapeSimLDBG_<< "shape INT param '"<<#name<<" = "<<name;}
    
int ShapeSim::startGrab(uint32_t _shots,void*_framebuf,cameraGrabCallBack _fn){
    int ret=-1;
    shots=_shots;
    //framebuf=_framebuf;
    fn=_fn;
    err_centerx=0;
    err_centery=0;
    err_sizex=0;
    err_sizey=0;

    err_rotangle=0;
    err_extangle=0;
    err_tickness=0;

    // fetch parameters
    if(shape_params->hasKey("framerate")){
        framerate=shape_params->getDoubleValue("framerate");
    }

    if(shape_params->hasKey("type")){
        shape_type=shape_params->getStringValue("type");
        ret =0;
        centerx=width/2;
        centery=height/2;
        rotangle=0;
        extangle=0;
        colr=255;
        colg=0;
        colb=0;
        tickness=2;
        linetype=8; //connected
        GETINTPARAM(shape_params,movex);
        GETINTPARAM(shape_params,movey);
        GETINTPARAM(shape_params,rot);

        GETINTPARAM(shape_params,centerx);
        tmp_centerx=centerx;
        GETINTPARAM(shape_params,centery);
        tmp_centery=centery;

        sizex=centerx/2;
        sizey=centery/2;

        GETDBLPARAM(shape_params,err_centerx);
        GETDBLPARAM(shape_params,err_centery);
        GETDBLPARAM(shape_params,err_sizex);
        GETDBLPARAM(shape_params,err_sizey);
        GETDBLPARAM(shape_params,err_rotangle);
        GETDBLPARAM(shape_params,err_extangle);
        GETDBLPARAM(shape_params,err_tickness);
        GETINTPARAM(shape_params,sizex);
        tmp_sizex=sizex;
        GETINTPARAM(shape_params,sizey);
        tmp_sizey=sizey;


        GETDBLPARAM(shape_params,rotangle);
        tmp_rotangle=rotangle;

        GETDBLPARAM(shape_params,extangle);

        GETINTPARAM(shape_params,tickness);
        tmp_tickness=tickness;

        GETINTPARAM(shape_params,linetype);
        GETINTPARAM(shape_params,colg);
        GETINTPARAM(shape_params,colb);
        GETINTPARAM(shape_params,colr);

    }
    last_acquisition_ts=chaos::common::utility::TimingUtil::getTimeStampInMicroseconds();
    ShapeSimLDBG_<<"Start Grabbing at:"<<framerate <<" frame/s";
    img.release();
    img.create(cv::Size(width,height),  CV_8UC3);
    return ret;
}
#define RND_DIST(var) \
    tmp_##var=var;\
    if(err_##var>0){\
    boost::random::uniform_int_distribution<> dist_##var(-err_##var,err_##var);\
    double err=dist_##var(gen);\
    tmp_##var+= err;}

int ShapeSim::waitGrab(const char**buf,uint32_t timeout_ms){
    int32_t ret=-1;
    size_t size_ret;
    boost::random::mt19937 gen(std::time(0) );
  //  img.zeros(cv::Size(height,width),  CV_8UC3);
    img.setTo(cv::Scalar::all(0));

    std::stringstream ss,fs;

    ss<<getUid()<<":"<<frames++;

    if(movex){
        if(((tmp_centerx+tmp_sizex+movex)>=width)|| ((tmp_centerx-tmp_sizex+movex)<=0)){
            movex=-movex;
        }
        tmp_centerx+=movex;
        
    } else {
        RND_DIST(centerx);
        RND_DIST(sizex);


    }

    if(movey){
        if(((tmp_centery+tmp_sizey+movey)>=height)|| ((tmp_centery-tmp_sizey+movey)<=0)){
            movey=-movey;
        }
        tmp_centery+=movex;
        
    } else {
        RND_DIST(centery);
        RND_DIST(sizey);
    }
    if(rot){
        tmp_rotangle+=rot;
    } else {
        RND_DIST(rotangle);
        RND_DIST(tickness);
    }
    fs<<shape_type<<":("<<tmp_centerx<<","<<tmp_centery<<") "<<tmp_sizex<<"x"<<tmp_sizey<<","<<tmp_rotangle<<","<<tmp_tickness;
    rectangle(img,Point( 0, 0),Point( width-1, height-1 ), Scalar( colr, colg, colb ));
    if(shape_type == "ellipse"){
        // get parameters
       
        
       

        ellipse( img,
                 Point( tmp_centerx, tmp_centery),
                 Size( tmp_sizex, tmp_sizey ),
                 tmp_rotangle,
                 0,
                 360,
                 Scalar( colr, colg, colb ),
                 tmp_tickness,
                 linetype );
        putText(img,ss.str(),Point(10,25),FONT_HERSHEY_SIMPLEX, 1,(255,255,255),1,LINE_AA);
        
        putText(img,fs.str(),Point(10,height-25),FONT_HERSHEY_SIMPLEX, 1,(255,255,255),1,LINE_AA);
        int size = img.total() * img.elemSize();
#ifdef CVDEBUG

        cv::imshow("test",img);
        imshow( "test", img );
        waitKey( 0 );
#endif
        
        if(framebuf_size[frames&1]<size){
            free(framebuf[frames&1]);
            framebuf[frames&1]=malloc(size);
            framebuf_size[frames&1]=size;
        }
        ShapeSimLDBG_<<ss.str()<<","<<fs.str()<<" size byte:"<<size<<" framerate:"<<framerate;
        std::memcpy(framebuf[frames&1],img.data,size );
    
        if(buf){
            if(frames>0){
                *buf=(char*)framebuf[!(frames&1)];
            } else {
                *buf=(char*)framebuf[0];
            }
        } else {
            ShapeSimLERR_<<"BAD BUFFER GIVEN "<<shape_type<<"("<<width<<"X"<<height<<")"<<frames<<" center "<<tmp_centerx<<","<<tmp_centery<<" sizex:"<<tmp_sizex<<" sizey:"<<tmp_sizey<<" color:"<<colr<<"R,"<<colg<<"G,"<<colb<<" size byte:"<<size;

        }
        
        
        ret=size;
    } else {
        ShapeSimLERR_<<"Unknown shape given:"<<shape_type;
    }
    if(framerate>0){
        uint64_t now=chaos::common::utility::TimingUtil::getTimeStampInMicroseconds();
        double diff=(1000000.0/framerate) - (now-last_acquisition_ts);
        last_acquisition_ts=now;
        if(diff>0){
	  boost::this_thread::sleep_for(boost::chrono::microseconds(diff));
	  
          
        }
        
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

int ShapeSim::waitGrab(uint32_t timeout_ms){
        ShapeSimLERR_<<"NOT IMPLEMENTED";

    return 0;
    //return waitGrab((const char**)&framebuf,timeout_ms);
}
int ShapeSim::stopGrab(){
    //camera->StopGrabbing();
    img.deallocate();
    img.release();
    return 0;
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
        val = cw->getInt32Value(propname);
        return 0;
    }
    return -1;
}

int  ShapeSim::getCameraProperty(const std::string& propname,double& val){
    ChaosUniquePtr<chaos::common::data::CDataWrapper> cw(new chaos::common::data::CDataWrapper());

    cameraToProps(cw.get());
    if(cw->hasKey(propname)){
        val = cw->getDoubleValue(propname);
        return 0;
    }
    return -1;
}

int  ShapeSim::getCameraProperties(chaos::common::data::CDataWrapper& proplist){

    return cameraToProps(&proplist);


}
}}}
