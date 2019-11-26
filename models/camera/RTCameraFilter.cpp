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
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#ifdef CERN_ROOT
#include <TH2F.h>
#include <TF2.h>
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
                   0),remove_src(false),apply_gauss_fit(false),h(NULL),g2d(NULL)
{
  RTCameraFilterLDBG_ << "Creating " << _control_unit_id
                      << " params:" << _control_unit_param;
  try
  {
    chaos::common::data::CDataWrapper p;
    p.setSerializedJsonData(_control_unit_param.c_str());

    apply_moment = false;
    moment_circle = 0;
    REFMOMENTX = -1;
    REFMOMENTY = -1;
    REFMOMENTRADIUS = 0;
    if (p.hasKey(FILTER_MOMENT_KEY))
    {
      moment_circle = p.getInt32Value(FILTER_MOMENT_KEY);
      apply_moment = true;
    }
    #ifdef CERN_ROOT
    if (p.hasKey(GAUSS_FIT_KEY))
    {
      gauss_fit_level=0;
      if(p.isCDataWrapperValue(GAUSS_FIT_KEY)){
        chaos::common::data::CDWUniquePtr gf=p.getCSDataValue(GAUSS_FIT_KEY);
        if(gf->hasKey("level")){
          gauss_fit_level=gf->getInt32Value("level");
        }
      }
      apply_gauss_fit = true;
    }
    #endif
  }
  catch (...)
  {
  }
}

/*
 Destructor
 */
RTCameraFilter::~RTCameraFilter() {
#ifdef CERN_ROOT
    if (apply_gauss_fit){
      if(g2d){
        delete g2d;
      
      }
    if(h){
      delete h;
    }

    }
#endif

}

#define SETINTPROP(name, n, value) \
  if (name == #n)                  \
  {                                \
    n = value;                     \
    return true;                   \
  }
bool RTCameraFilter::setProp(const std::string &name,  chaos::common::data::CDataWrapper p,
                             uint32_t size)
{

  if (name != "FILTER")
  {
    return false;
  }
  
  if(p.hasKey(FILTER_MOMENT_KEY)&&p.isInt32Value(FILTER_MOMENT_KEY)){
      moment_circle = p.getInt32Value(FILTER_MOMENT_KEY);
      apply_moment = true;
    } else {
      apply_moment = true;
    }
    if(p.hasKey(FILTER_REMOVE_SOURCE_KEY)&&p.isBoolValue(FILTER_REMOVE_SOURCE_KEY)){
      remove_src=p.getBoolValue(FILTER_REMOVE_SOURCE_KEY);
    }
  
 
  return true;
}

bool RTCameraFilter::setProp(const std::string &name, int32_t value,
                             uint32_t size)
{
  int ret;
  int32_t valuer;
  RTCameraFilterLDBG_ << "SET IPROP:" << name << " VALUE:" << value;
  SETINTPROP(name, REFMOMENTX, value);
  SETINTPROP(name, REFMOMENTY, value);
  SETINTPROP(name, REFMOMENTRADIUS, value);

  return RTCameraBase::setProp(name, value, size);
}

bool RTCameraFilter::setProp(const std::string &name, double value,
                             uint32_t size)
{
  return RTCameraBase::setProp(name, value, size);
}
//! Return the definition of the control unit
/*!
The api that can be called withi this method are listed into
"Control Unit Definition Public API" module into html documentation
(chaosframework/Documentation/html/group___control___unit___definition___api.html)
*/
void RTCameraFilter::unitDefineActionAndDataset() throw(chaos::CException)
{

  addAttributeToDataSet("FILTER", "Filters to apply (JSON)",
                        chaos::DataType::TYPE_CLUSTER, chaos::DataType::Input);

  addHandlerOnInputAttributeName<::driver::sensor::camera::RTCameraFilter,
                                  chaos::common::data::CDataWrapper>(
      this, &::driver::sensor::camera::RTCameraFilter::setProp, std::string("FILTER"));

  if (apply_moment)
  {
    addAttributeToDataSet("MOMENTX", "Centroid X", chaos::DataType::TYPE_INT32,
                          chaos::DataType::Output);
    addAttributeToDataSet("MOMENTY", "Centroid Y", chaos::DataType::TYPE_INT32,
                          chaos::DataType::Output);
    addAttributeToDataSet("MOMENTERR", "Distance from reference and current",
                          chaos::DataType::TYPE_DOUBLE,
                          chaos::DataType::Output);

    addAttributeToDataSet("REFMOMENTX", "Ref Centroid X",
                          chaos::DataType::TYPE_INT32, chaos::DataType::Input);
    addAttributeToDataSet("REFMOMENTY", "Ref Centroid Y",
                          chaos::DataType::TYPE_INT32, chaos::DataType::Input);

    addAttributeToDataSet("REFMOMENTRADIUS",
                          "Generate Error if outside this radius",
                          chaos::DataType::TYPE_INT32, chaos::DataType::Input);
    addStateVariable(StateVariableTypeAlarmCU, "moment_out_of_set",
                     "moment out of reference moment radius");
  }
  if(apply_gauss_fit){
    addAttributeToDataSet("Amplitude", "Fit Amplitude ", chaos::DataType::TYPE_DOUBLE,
                          chaos::DataType::Output);
    
    addAttributeToDataSet("X_m", "Fit X average", chaos::DataType::TYPE_DOUBLE,
                          chaos::DataType::Output);
    addAttributeToDataSet("Y_m", "Fit Y average", chaos::DataType::TYPE_DOUBLE,
                          chaos::DataType::Output);
    
    addAttributeToDataSet("S_x", "Fit Sigma X ", chaos::DataType::TYPE_DOUBLE,
                          chaos::DataType::Output);
    addAttributeToDataSet("S_y", "Fit Sigma Y ", chaos::DataType::TYPE_DOUBLE,
                          chaos::DataType::Output);
    addAttributeToDataSet("rho", "Fit Tilt angle ", chaos::DataType::TYPE_DOUBLE,
                          chaos::DataType::Output);
    
    addAttributeToDataSet("Y_m", "Fit Y average", chaos::DataType::TYPE_DOUBLE,
                          chaos::DataType::Output);
    
    
  }
  RTCameraBase::unitDefineActionAndDataset();
}

int RTCameraFilter::filtering(cv::Mat &image)
{
    Mat thr, gray;
  if(apply_moment || apply_gauss_fit){
// convert image to grayscale
    cvtColor(image, gray, COLOR_BGR2GRAY);

    // convert grayscale to binary image
    threshold(gray, thr, 100, 255, THRESH_BINARY);

  }
  if(apply_gauss_fit){
#ifdef CERN_ROOT
    if(h==NULL){
     h = new TH2F("slm"," ",gray.cols,0,gray.cols,gray.rows,0,gray.rows);
     h->SetStats(0);

    }
    if(g2d==NULL){
      g2d = new TF2("g2d",beamFunc,0.,gray.cols,0.,gray.rows,6);
      //g2d->Draw("SAME");
      g2d->SetParNames("Amplitude","X_m","S_x","Y_m","S_y","rho");

    }
      int xPixels=gray.cols;
      int yPixels=gray.rows;
      for (int row=0; row<xPixels; ++row) {
        for (int col=0; col<yPixels; ++col) {
            //float greyv = (255.0*1.0-gray.at<uchar>(row,col))/256.0;
            float greyv = (gray.at<uchar>(col,row))/256.0;
            //if (row<5 || row > xPixels-5 || col < 5 || col>yPixels-5) {
            //    grey = 0;
            //}
            //printf("%d(%d,%d) -> %f\n",index,row,col,grey);
            h->SetBinContent(row,col,greyv);
            //  h->SetBinError(row+1,yPixels-col,TMath::Sqrt(grey));
        }
    }
    double Amplitude =h->GetMaximum();
    double X_m       =h->GetMean(1);
    double S_x       = h->GetRMS(1);
    double Y_m       = h->GetMean(2);
    double S_y       = h->GetRMS(2);
    double rho       = h->GetCorrelationFactor();
    double par[6] = {Amplitude,X_m,S_x,Y_m,S_y,rho};

    if(gauss_fit_level>0){
      g2d->SetParameters(par);
      
      RTCameraFilterLDBG_ << "Start Fitting params: A:"<<par[0]<<" Mx:"<<par[1]<<" Sx:"<<par[2]<<" My:"<<par[3]<<" Sy:"<<par[4]<<" rho:"<<par[5];

      h->Fit("g2d","N","");
     Amplitude = g2d->GetParameter(0);
     X_m       = g2d->GetParameter(1);
     S_x       = g2d->GetParameter(2);
     Y_m       = g2d->GetParameter(3);
     S_y       = g2d->GetParameter(4);
     rho       = g2d->GetParameter(5);

    } 
   

    RTCameraFilterLDBG_ << "End Fitting C:"<<X_m<<","<<Y_m<<" S:"<<S_x<<","<<S_y<<" angle:"<<rho;

    getAttributeCache()->setOutputAttributeValue("Amplitude",Amplitude);
    getAttributeCache()->setOutputAttributeValue("X_m",X_m);
    getAttributeCache()->setOutputAttributeValue("S_x",S_x);
    getAttributeCache()->setOutputAttributeValue("Y_m",Y_m);
    getAttributeCache()->setOutputAttributeValue("S_y",S_y);
    getAttributeCache()->setOutputAttributeValue("rho",rho);

    

#endif
  }
  if (apply_moment)
  {
    
    // find moments of the image
    Moments m = moments(thr, true);
    Point p(m.m10 / m.m00, m.m01 / m.m00);
    getAttributeCache()->setOutputAttributeValue("MOMENTX", p.x);
    getAttributeCache()->setOutputAttributeValue("MOMENTY", p.y);

    if (moment_circle > 0)
    {
      circle(image, p, moment_circle, Scalar(128, 0, 0), -1);
    }
    if ((REFMOMENTX > 0) && (REFMOMENTY > 0))
    {
      double dist = sqrt(pow((p.x - REFMOMENTX), 2) + pow((p.y - REFMOMENTY), 2));
      getAttributeCache()->setOutputAttributeValue("MOMENTERR", dist);
      if (REFMOMENTRADIUS > 0)
      {
        if (dist > REFMOMENTRADIUS)
        {
          circle(image, Point(REFMOMENTX, REFMOMENTY), REFMOMENTRADIUS, Scalar(128, 0, 0), 1);

          setStateVariableSeverity(StateVariableTypeAlarmCU, "moment_out_of_set",
                                   chaos::common::alarm::MultiSeverityAlarmLevelWarning);
        }
        else
        {
          circle(image, Point(REFMOMENTX, REFMOMENTY), REFMOMENTRADIUS, Scalar(0, 255, 0), 1);

          setStateVariableSeverity(StateVariableTypeAlarmCU, "moment_out_of_set",
                                   chaos::common::alarm::MultiSeverityAlarmLevelClear);
        }
      }
    }
  }

  return 0;
}
