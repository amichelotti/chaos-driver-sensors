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
#include <boost/random/uniform_real.hpp>
#include <chaos/common/data/structured/DatasetAttribute.h>


#include "../beamFunc.h"

namespace cu_driver = chaos::cu::driver_manager::driver;
using namespace ::driver::sensor::camera;
using namespace chaos::common::data::structured;


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
    throw chaos::CException(-1,"cannot intialize camera expected bad JSON parameters"+std::string(initParameter),__PRETTY_FUNCTION__);
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
    propsToCamera(( chaos::common::data::CDataWrapper*)&json);
    original_width=width;
    original_height=height;
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
DEFAULT_CU_DRIVER_PLUGIN_CONSTRUCTOR_WITH_NS(::driver::sensor::camera,ShapeSim),shots(0),frames(0),fn(NULL),props(NULL),pixelEncoding(CV_8UC3),tmode(CAMERA_TRIGGER_CONTINOUS),gstrategy(CAMERA_LATEST_ONLY),initialized(false),height(CAM_DEFAULT_HEIGTH),width(CAM_DEFAULT_WIDTH),framerate(1.0),offsetx(0),offsety(0){

    ShapeSimLDBG_<<  "Created Driver";
    framebuf[0]=NULL;
    framebuf[1]=NULL;
    framebuf_size[0]=0;
    framebuf_size[1]=0;
    movex=movey=rot=0;
    max_movex=max_movey=min_movex=min_movey=0;
    amplitude=1.0;
    err_amplitude=0;
    sigmax=sigmay=10.0;
    gain_raw=600;
    brightness_raw=500;
    shutter_raw=500;
    gain=1.0;
    shutter=1.0;
    brightness=1.0;
    trigger_mode=0;
    props=new chaos::common::data::CDataWrapper();
    ownprops->reset();
    ownprops->createProperty("ExposureTimeRaw",shutter_raw,0,CAM_MAX_SHUTTER,1,"SHUTTER");
    ownprops->createProperty("GainRaw",gain_raw,0,CAM_MAX_GAIN,1,"GAIN");
    ownprops->createProperty("BslBrightnessRaw",brightness_raw,0,CAM_MAX_BRIGHTNESS,1,"BRIGHTNESS");
#ifdef CVDEBUG

    cv::namedWindow("test");
#endif
ShapeSimLDBG_<<" Internal property:"<<ownprops->getProperty().getJSONString();
}
//default descrutcor
ShapeSim::~ShapeSim() {

}

int ShapeSim::cameraToProps(chaos::common::data::CDataWrapper*p){

    if(p == NULL){
        p = props;
    }
    /*DatasetAttribute speed("speed", "Max speed of trapezoidal profile",
                         chaos::DataType::DataType::TYPE_DOUBLE,
                         chaos::DataType::Input, "0.001", "500.0", "400.0",
                         "0.1", "mm/s");

*/
    ShapeSimLDBG_<< "WIDTH:"<<width<<" HEIGHT:"<<height;
    ShapeSimLDBG_<< "FRAMERATE:"<<framerate;
    p->addDoubleValue("FRAMERATE",framerate);
    p->addInt32Value("WIDTH",width);
    p->addInt32Value("HEIGHT",height);

    p->addInt32Value("TRIGGER_MODE",trigger_mode);
    p->addDoubleValue("GAIN",gain);
    p->addDoubleValue("BRIGHTNESS",brightness);
    p->addDoubleValue("SHUTTER",shutter);
    p->addInt32Value("OFFSETX",offsetx);
    p->addInt32Value("OFFSETY",offsety);
   
    p->addCSDataValue("custom",ownprops->getProperty());
    ShapeSimLDBG_<<" Properties:"<<p->getJSONString();

    return 0;
}

#define PXL2COLOR(col) \
    if(fmt == #col) {\
    color=col;}

int ShapeSim::propsToCamera(chaos::common::data::CDataWrapper*p){
    // Get the camera control object.
    int32_t ret=0;

    ShapeSimLDBG_<<"setting props: " << p->getCompliantJSONString() ;

    // Get the parameters for setting the image area of interest (Image AOI).

    // Maximize the Image AOI.
    //   chaos::common::data::CDataWrapper*p=driver->props;

    if (p->hasKey("OFFSETX")){
        offsetx=p->getInt32Value("OFFSETX");
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
    if (p->hasKey(FRAMEBUFFER_ENCODING_KEY)&&p->isStringValue(FRAMEBUFFER_ENCODING_KEY)){
        ShapeSimLDBG_<< "setting FRAMEBUFFER_ENCODING " << p->getStringValue(FRAMEBUFFER_ENCODING_KEY);

        pixelEncoding=fmt2cv(p->getStringValue(FRAMEBUFFER_ENCODING_KEY));
    }

    if(p->hasKey("GAIN")){
        gain_raw=p->getDoubleValue("GAIN")*CAM_MAX_GAIN/100;
        ShapeSimLDBG_<< "SETTING GAIN RAW:"<<gain_raw <<" norm:"<<p->getDoubleValue("GAIN");
        gain=p->getDoubleValue("GAIN");
    }

    if(p->hasKey("SHUTTER")){
        shutter_raw=(p->getDoubleValue("SHUTTER")*CAM_MAX_SHUTTER)/100;
        ShapeSimLDBG_<< "SETTING SHUTTER RAW:"<<shutter_raw <<" norm:"<<p->getDoubleValue("SHUTTER");
        shutter=p->getDoubleValue("SHUTTER");
    }
    if(p->hasKey("ZOOM")){

    }
    // Pixel Clock

    if(p->hasKey("FRAMERATE")){
        framerate=p->getDoubleValue("FRAMERATE");
        ShapeSimLDBG_<< "FRAME RATE:"<<framerate;
    }
    if(p->hasKey("TRIGGER_MODE")){
        trigger_mode=p->getInt32Value("TRIGGER_MODE");
    }

    if(p->hasKey("SHARPNESS")){
    }
    if(p->hasKey("BRIGHTNESS")){
        brightness_raw=p->getDoubleValue("BRIGHTNESS")*CAM_MAX_BRIGHTNESS/100;
        ShapeSimLDBG_<< "SETTING BRIGHTNESS RAW:"<<brightness_raw <<" norm:"<<p->getDoubleValue("BRIGHTNESS");
        brightness=p->getDoubleValue("BRIGHTNESS");
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
    
void ShapeSim::createBeamImage(Size size, Mat& output,float uX,float uY, float sx, float sy, float amp,float angle){
    Mat temp=Mat(size,CV_32F);
    double p[6];
    double x[2];

    p[0]=amp;
    p[1]=uX;
    p[2]=sx;
    p[3]=uY;
    p[4]=sy;
    p[5]=angle;
   
    for(int r=0;r<size.height;r++){
        for(int c=0;c<size.width;c++){
            x[0]=c;
            x[1]=r;
            temp.at<float>(r,c)=beamFunc(x,p);
            
        }
    }
    ShapeSimLDBG_<<"Gaussian("<<uX<<","<<uY<<") sx:"<<sx<<" sy:"<<sy<<" amp:"<<amp;

    normalize(temp,temp,0.0,255,NORM_MINMAX);
    if(pixelEncoding==CV_8UC3){
        temp.convertTo(temp,CV_8U);
        cvtColor(temp, output, CV_GRAY2BGR);
    } else if(pixelEncoding==CV_8UC1){
            temp.convertTo(output,CV_8U);

    } else {
        throw chaos::CException(-4,"Unsupported Encoding ",__PRETTY_FUNCTION__);

    }
}
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
        err_sigmax=err_sigmay=rho=0;
        GETDBLPARAM(shape_params,movex);
        GETDBLPARAM(shape_params,movey);
        GETDBLPARAM(shape_params,rot);

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
         max_movex=width-tmp_sizex;
         max_movey=height-tmp_sizey;
         min_movex=tmp_sizex;
         min_movey=tmp_sizey;
         max_amplitude=inc_amplitude=0;
        GETDBLPARAM(shape_params,max_movex);
        GETDBLPARAM(shape_params,max_movey);
        GETDBLPARAM(shape_params,min_movey);
        GETDBLPARAM(shape_params,min_movex);

        GETDBLPARAM(shape_params,amplitude);
        tmp_amplitude=amplitude;
        max_amplitude=amplitude;
        GETDBLPARAM(shape_params,sigmax);
        GETDBLPARAM(shape_params,sigmay);
        GETDBLPARAM(shape_params,err_sigmay);
        GETDBLPARAM(shape_params,err_sigmax);
        GETDBLPARAM(shape_params,max_amplitude);
        GETDBLPARAM(shape_params,inc_amplitude);
        GETDBLPARAM(shape_params,rho);



    }
    last_acquisition_ts=chaos::common::utility::TimingUtil::getTimeStampInMicroseconds();
    ShapeSimLDBG_<<"Start Grabbing at:"<<framerate <<" frame/s";
    img.release();
    img.create(cv::Size(original_width,original_height),  pixelEncoding);
    return ret;
}
#define RND_DIST(var) \
    tmp_##var=var;\
    if(err_##var>0){\
    boost::random::uniform_int_distribution<> dist_##var(-err_##var,err_##var);\
    double err=dist_##var(gen);\
    tmp_##var+= err;}

static double getError(double mxerror,boost::random::mt19937& gen){
        if(mxerror==0) return 0;
        boost::random::uniform_real_distribution<> dist_err(-mxerror,mxerror);
        return dist_err(gen);

}
void ShapeSim::applyCameraParams(cv::Mat& image){
    if(gain_raw<0){
        gain_raw=CAM_MAX_GAIN;
    }
    if(brightness_raw<0){
        brightness_raw=CAM_MAX_BRIGHTNESS;
    }
    if(shutter_raw<0){
        shutter_raw=CAM_MAX_SHUTTER;
     }
    
    for( int y = 0; y < image.rows; y++ ) {
        for( int x = 0; x < image.cols; x++ ) {
            for( int c = 0; c < image.channels(); c++ ) {
                image.at<Vec3b>(y,x)[c] =
                  saturate_cast<uchar> ( (((double)1.0*gain_raw)/CAM_MAX_GAIN*image.at<Vec3b>(y,x)[c] + (((double)1.0*brightness_raw))/CAM_MAX_BRIGHTNESS)*(((double)1.0*shutter_raw)/CAM_MAX_SHUTTER) );
            }
        }
    }
}

int ShapeSim::waitGrab(const char**buf,uint32_t timeout_ms){
    int32_t ret=-1;
    size_t size_ret;
    int size=0;
    boost::random::mt19937 gen(std::time(0) );
  //  img.zeros(cv::Size(height,width),  CV_8UC3);
    img.setTo(cv::Scalar::all(0));

    std::stringstream ss,fs;

    ss<<getUid()<<":"<<frames++;

    if(movex){
        if(((tmp_centerx+movex)>=max_movex)|| ((tmp_centerx+movex)<=min_movex)){
            movex=-movex;
        }
        tmp_centerx+=movex;
        
    } else {
        RND_DIST(centerx);
        RND_DIST(sizex);


    }

    if(movey){
        if(((tmp_centery+movey)>=max_movey)|| ((tmp_centery+movey)<=min_movey)){
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
    if(shape_type=="beam"){
        if(inc_amplitude>0){
            if(tmp_amplitude<(amplitude+max_amplitude)){
                tmp_amplitude+=inc_amplitude;
            } else {
                tmp_amplitude=amplitude;
            }

        } else {
            tmp_amplitude=amplitude;

        }
        createBeamImage(Size(original_width,original_height),img,
        tmp_centerx+getError((double)err_centerx,gen),
        tmp_centery+getError((double)err_centery,gen),
        sigmax+getError(err_sigmax,gen),
        sigmay+getError(err_sigmay,gen),
        tmp_amplitude+getError(err_amplitude,gen));
    } else if(shape_type == "ellipse"){
        // get parameters
       
        fs<<shape_type<<":("<<tmp_centerx<<","<<tmp_centery<<") "<<tmp_sizex<<"x"<<tmp_sizey<<","<<tmp_rotangle<<","<<tmp_tickness;

        rectangle(img,Point( 0, 0),Point( width-1, height-1 ), Scalar( colr, colg, colb ));


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
        } else {
        ShapeSimLERR_<<"Unknown shape given:"<<shape_type;
        }
        cv::Mat cropped;
        if(((offsetx+width) <= img.cols)&&((offsety+height)<img.rows)){
            Rect region_of_interest = Rect(offsetx, offsety, width, height);
            Mat image_roi = img(region_of_interest);
            image_roi.copyTo(cropped);
        } else {
            cropped=img;
        }

        applyCameraParams(cropped);
         size = cropped.total() * cropped.elemSize();

#ifdef CVDEBUG

        cv::imshow("test",img);
        imshow( "test", img );
        waitKey( 0 );
#endif
        
        if(framebuf_size[frames&1]<size){
            //free(framebuf[frames&1]);
            framebuf[frames&1]=realloc(framebuf[frames&1],size);
            framebuf_size[frames&1]=size;
        }
        if(size>0 && cropped.data &&framebuf[frames&1]){
            ShapeSimLDBG_<<ss.str()<<","<<fs.str()<<" size byte:"<<size<<" framerate:"<<framerate;
            std::memcpy(framebuf[frames&1],cropped.data,size );
    
        if(buf){
            if(frames>0){
                *buf=(char*)framebuf[!(frames&1)];
            } else {
                *buf=(char*)framebuf[0];
            }
        } else {
            ShapeSimLERR_<<"BAD BUFFER GIVEN "<<shape_type<<"("<<width<<"X"<<height<<")"<<frames<<" center "<<tmp_centerx<<","<<tmp_centery<<" sizex:"<<tmp_sizex<<" sizey:"<<tmp_sizey<<" color:"<<colr<<"R,"<<colg<<"G,"<<colb<<" size byte:"<<size;

        }
        }
        
        ret=size;
    
    if(framerate>0){
        uint64_t now=chaos::common::utility::TimingUtil::getTimeStampInMicroseconds();
        double diff=(1000000.0/framerate) - (now-last_acquisition_ts);
        last_acquisition_ts=now;
        if(diff>0){
	  boost::this_thread::sleep_for(boost::chrono::microseconds((uint64_t)diff));
	  
          
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
