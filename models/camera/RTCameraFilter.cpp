/*
 *	RTCameraFilter.cpp
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

#include "RTCameraFilter.h"
#include <chaos/cu_toolkit/control_manager/AttributeSharedCacheWrapper.h>
#include <driver/sensors/core/AbstractSensorDriver.h>
#include <driver/sensors/core/CameraDriverInterface.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <math.h>
#ifdef CERN_ROOT
#include <TF2.h>
#include <TH2F.h>
#endif
#include "beamFunc.h"

using namespace chaos;
using namespace chaos::common::data::cache;
using namespace chaos::cu::driver_manager::driver;
using namespace ::driver::sensor::camera;
using namespace chaos::cu::control_manager;
using namespace cv;
using namespace boost::interprocess;

PUBLISHABLE_CONTROL_UNIT_IMPLEMENTATION(RTCameraFilter)

/*#define RTCameraFilterLAPP_		LAPP_ << "[RTCameraFilter] "
#define RTCameraFilterLDBG_		LDBG_ << "[RTCameraFilter] " <<
__PRETTY_FUNCTION__ << " " #define RTCameraFilterLERR_		LERR_ <<
"[RTCameraFilter] " << __PRETTY_FUNCTION__ << "("<<__LINE__<<") "
*/
#define RTCameraFilterLAPP_ CUAPP
#define RTCameraFilterLDBG_ CUDBG
#define RTCameraFilterLERR_ CUERR
/*
 Construct
 */

RTCameraFilter::RTCameraFilter(
    const string &_control_unit_id, const string &_control_unit_param,
    const ControlUnitDriverList &_control_unit_drivers)
    : RTCameraBase(_control_unit_id, _control_unit_param, _control_unit_drivers,
                   0),
      remove_src(false), apply_gauss_fit(false), h(NULL), g2d(NULL) {
  RTCameraFilterLDBG_ << "Creating " << _control_unit_id
                      << " params:" << _control_unit_param;
  try {
    chaos::common::data::CDataWrapper p;
    p.setSerializedJsonData(_control_unit_param.c_str());

    
    Amplitude = X_m = S_x = Y_m = S_y = rho = 0;
   apply_gauss_fit=true; 
  } catch (...) {
  }
  gauss_fit_level=0;
  CREATE_CU_INT_PROP(FILTER_MOMENT_KEY, "", gauss_fit_level, 0, 1024, 1,
                     RTCameraFilter);
  CREATE_CU_BOOL_PROP("apply_gauss_fit", "", apply_gauss_fit, RTCameraFilter);
}

/*
 Destructor
 */
RTCameraFilter::~RTCameraFilter() {
#ifdef CERN_ROOT
  if (apply_gauss_fit) {
    if (g2d) {
      delete g2d;
    }
    if (h) {
      delete h;
    }
  }
#endif
}

#define SETINTPROP(name, n, value)                                             \
  if (name == #n) {                                                            \
    n = value;                                                                 \
    return true;                                                               \
  }


/*
bool RTCameraFilter::setDrvProp(const std::string &name, int32_t value,
                                uint32_t size) {
  int ret;
  int32_t valuer;
  RTCameraFilterLDBG_ << "SET IPROP:" << name << " VALUE:" << value;
  

  return RTCameraBase::setDrvProp(name, value, size);
}
*/
//bool RTCameraFilter::setDrvProp(const std::string &name, double value,
//                                uint32_t size) {
/*  SETINTPROP(name, REFX, value);
  SETINTPROP(name, REFY, value);
  SETINTPROP(name, REFSX, value);
  SETINTPROP(name, REFSY, value);
  SETINTPROP(name, REFRHO, value);
*/
  //return RTCameraBase::setDrvProp(name, value, size);

//}

//! Return the definition of the control unit
/*!
The api that can be called withi this method are listed into
"Control Unit Definition Public API" module into html documentation
(chaosframework/Documentation/html/group___control___unit___definition___api.html)
*/

void RTCameraFilter::unitInit() throw(chaos::CException) {

  AttributeSharedCacheWrapper *cc = getAttributeCache();
  
  
  REFX = cc->getRWPtr<double>(DOMAIN_INPUT, "REFX");
  REFY = cc->getRWPtr<double>(DOMAIN_INPUT, "REFY");
  REFSX = cc->getRWPtr<double>(DOMAIN_INPUT, "REFSX");
  REFSY = cc->getRWPtr<double>(DOMAIN_INPUT, "REFSY");
  REFRHO = cc->getRWPtr<double>(DOMAIN_INPUT, "REFRHO");
  RTCameraBase::unitInit();
}
void RTCameraFilter::unitDefineActionAndDataset() throw(chaos::CException) {

  
    
    addAttributeToDataSet("REFX", "Reference X ", chaos::DataType::TYPE_DOUBLE,
                          chaos::DataType::Input);
    addAttributeToDataSet("REFY", "Reference Y", chaos::DataType::TYPE_DOUBLE,
                          chaos::DataType::Input);

    addAttributeToDataSet("REFSX", "Reference Sigma X ", chaos::DataType::TYPE_DOUBLE,
                          chaos::DataType::Input);
    addAttributeToDataSet("REFSY", "Reference Sigma Y ", chaos::DataType::TYPE_DOUBLE,
                          chaos::DataType::Input);
    addAttributeToDataSet("REFRHO", "Reference Tilt angle ",
                          chaos::DataType::TYPE_DOUBLE,
                          chaos::DataType::Input);
    
   /* 
   addHandlerOnInputAttributeName<::driver::sensor::camera::RTCameraFilter,
                                   double>(
        this, &::driver::sensor::camera::RTCameraFilter::setDrvProp,
        "REFX");
    addHandlerOnInputAttributeName<::driver::sensor::camera::RTCameraFilter,
                                   double>(
        this, &::driver::sensor::camera::RTCameraFilter::setDrvProp,
        "REFY");

    addHandlerOnInputAttributeName<::driver::sensor::camera::RTCameraFilter,
                                   double>(
        this, &::driver::sensor::camera::RTCameraFilter::setDrvProp,
        "REFSX");
    addHandlerOnInputAttributeName<::driver::sensor::camera::RTCameraFilter,
                                   double>(
        this, &::driver::sensor::camera::RTCameraFilter::setDrvProp,
        "REFSY");
addHandlerOnInputAttributeName<::driver::sensor::camera::RTCameraFilter,
                                   double>(
        this, &::driver::sensor::camera::RTCameraFilter::setDrvProp,
        "REFRHO");
*/
    addAttributeToDataSet("AMPLITUDE", "Estimated amplitude ", chaos::DataType::TYPE_DOUBLE,
                          chaos::DataType::Output);
    addAttributeToDataSet("X", "Estimated X ", chaos::DataType::TYPE_DOUBLE,
                          chaos::DataType::Output);
    addAttributeToDataSet("Y", "Estimated Y", chaos::DataType::TYPE_DOUBLE,
                          chaos::DataType::Output);

    addAttributeToDataSet("SX", "Estimated Sigma X ", chaos::DataType::TYPE_DOUBLE,
                          chaos::DataType::Output);
    addAttributeToDataSet("SY", "Estimated Sigma Y ", chaos::DataType::TYPE_DOUBLE,
                          chaos::DataType::Output);
    addAttributeToDataSet("RHO", "Estimated Tilt angle ",
                          chaos::DataType::TYPE_DOUBLE,
                          chaos::DataType::Output);

    
  addStateVariable(StateVariableTypeAlarmCU, "out_of_reference",
                     "estimated ellipse out of reference");
  
addStateVariable(StateVariableTypeAlarmCU, "filtering_error",
                     "Error performing filter");
  RTCameraBase::unitDefineActionAndDataset();
}
int checkPointInside(int x,int y,int angle,int sx,int sy,int px,int py){
  double ret=pow(((px-x)*cos(angle) + sin(angle)*(py-y)),2)/pow(sx,2) + pow(((px-x)*sin(angle) + cos(angle)*(py-y)),2)/pow(sy,2);
  return (ret<=1.0);
}
int checkEllipseInside(int x,int y,int angle,int sx,int sy,int px,int py,int psx,int psy, int pangle){
  double ret;
  // check the center is inside
  ret=checkPointInside(x,y,angle,sx,sy,px, py);
  if(ret>1) return false;
  int farestx=px+ cos(pangle)*psx;
  int faresty=py +sin(pangle)*psy;

  ret=checkPointInside(x,y,angle,sx,sy,farestx, faresty);
  if(ret>1) return false;
   farestx=px- cos(pangle)*psx;
   faresty=py-sin(pangle)*psy;

  ret=checkPointInside(x,y,angle,sx,sy,farestx, faresty);
  if(ret>1) return false;
  return (ret<=1.0);
}

int RTCameraFilter::filtering(cv::Mat &image) {
  try {
    Mat thr, gray;
    if (image.data == NULL) {
      RTCameraFilterLERR_ << " BAD IMAGE";
      return -1;
    }
    RTCameraFilterLDBG_ << " NCHANNELS:" << image.channels()
                        << " size:" << image.rows << "x" << image.cols;
    if (( apply_gauss_fit)) {
      if (image.channels() > 1) {
        // convert image to grayscale
        cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        // gray=image;

      } else {
        gray = image;
      }
    }
    if ( !apply_gauss_fit) {
      // convert grayscale to binary image
      threshold(gray, thr, 100, 255, THRESH_BINARY);
    }
    if (apply_gauss_fit) {
#ifdef CERN_ROOT
      if (h == NULL) {
        h = new TH2F("slm", " ", gray.cols, 0, gray.cols, gray.rows, 0,
                     gray.rows);
        h->SetStats(0);
      }
      if (g2d == NULL) {
        g2d = new TF2("g2d", beamFunc, 0., gray.cols, 0., gray.rows, 6);
        // g2d->Draw("SAME");
        g2d->SetParNames("Amplitude", "X_m", "S_x", "Y_m", "S_y", "rho");
      }
      int xPixels = gray.cols;
      int yPixels = gray.rows;
      for (int row = 0; row < xPixels; ++row) {
        for (int col = 0; col < yPixels; ++col) {
          // float greyv = (255.0*1.0-gray.at<uchar>(row,col))/256.0;
          float greyv = (gray.at<uchar>(col, row)) / 256.0;
          // if (row<5 || row > xPixels-5 || col < 5 || col>yPixels-5) {
          //    grey = 0;
          //}
          // printf("%d(%d,%d) -> %f\n",index,row,col,grey);
          h->SetBinContent(row, col, greyv);
          //  h->SetBinError(row+1,yPixels-col,TMath::Sqrt(grey));
        }
      }
      Amplitude = h->GetMaximum();
      X_m = h->GetMean(1);
      S_x = h->GetRMS(1);
      Y_m = h->GetMean(2);
      S_y = h->GetRMS(2);
      rho = h->GetCorrelationFactor();
      double par[6] = {Amplitude, X_m, S_x, Y_m, S_y, rho};

      if (gauss_fit_level > 0) {
        g2d->SetParameters(par);

        RTCameraFilterLDBG_ << "Start Fitting params: A:" << par[0]
                            << " Mx:" << par[1] << " Sx:" << par[2]
                            << " My:" << par[3] << " Sy:" << par[4]
                            << " rho:" << par[5];

        h->Fit("g2d", "N", "");
        Amplitude = g2d->GetParameter(0);
        X_m = g2d->GetParameter(1);
        S_x = g2d->GetParameter(2);
        Y_m = g2d->GetParameter(3);
        S_y = g2d->GetParameter(4);
        rho = g2d->GetParameter(5);
      }

      RTCameraFilterLDBG_ << "End Fitting C:" << X_m << "," << Y_m
                          << " S:" << S_x << "," << S_y << " angle:" << rho;

      getAttributeCache()->setOutputAttributeValue("AMPLITUDE", Amplitude);
      getAttributeCache()->setOutputAttributeValue("X", X_m);
      getAttributeCache()->setOutputAttributeValue("SX", S_x);
      getAttributeCache()->setOutputAttributeValue("Y", Y_m);
      getAttributeCache()->setOutputAttributeValue("SY", S_y);
      getAttributeCache()->setOutputAttributeValue("RHO", rho);

#endif
    }
    if (apply_gauss_fit) {
      
      if ((S_x > 0) && (S_y > 0)) {
        ellipse(image, Point(X_m, Y_m), Size(S_x, S_y), rho, 0, 360,
                Scalar(128, 0, 0), 1, 8);
      }

      if ((*REFSX > 0) && (*REFSY  > 0)) {
     setStateVariableSeverity(
            StateVariableTypeAlarmCU, "out_of_reference",
            chaos::common::alarm::MultiSeverityAlarmLevelClear);
        ellipse(image, Point(*REFSX, *REFSY), Size(*REFSX, *REFSX), *REFRHO, 0, 360,
                Scalar(255, 255, 255), 1, 8);

        if(checkEllipseInside(*REFX,*REFY,*REFRHO,*REFSX,*REFSY,X_m,Y_m,S_x,S_y,rho)==false){
setStateVariableSeverity(
            StateVariableTypeAlarmCU, "out_of_reference",
            chaos::common::alarm::MultiSeverityAlarmLevelHigh);
        }
      }
    }
       
    setStateVariableSeverity(StateVariableTypeAlarmCU, "filtering_error",chaos::common::alarm::MultiSeverityAlarmLevelClear);
  } catch (cv::Exception &ex) {
    std::stringstream ss;
 setStateVariableSeverity(StateVariableTypeAlarmCU, "filtering_error",
            chaos::common::alarm::MultiSeverityAlarmLevelHigh);
    ss << "OpenCV Exception filtering image:" << ex.what();
    RTCameraFilterLERR_ << ss.str();
    return -2;

  } catch (runtime_error &ex) {
    std::stringstream ss;

    ss << "Exception Filtering image:" << ex.what();
  setStateVariableSeverity(StateVariableTypeAlarmCU, "filtering_error",chaos::common::alarm::MultiSeverityAlarmLevelHigh);
    RTCameraFilterLERR_ << ss.str();
    return -3;
    //  metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelError,ss.str());
  }
  return 0;
}
