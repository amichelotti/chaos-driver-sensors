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
                   0),
      remove_src(false), apply_gauss_fit(false), h(NULL), g2d(NULL)
{
  RTCameraFilterLDBG_ << "Creating " << _control_unit_id
                      << " params:" << _control_unit_param;
  try
  {
    chaos::common::data::CDataWrapper p;
    p.setSerializedJsonData(_control_unit_param.c_str());

    apply_moment = false;
    moment_circle = 0;
    REFOFFSETX = 0;
    REFOFFSETY = 0;
    REFSIZEX = 0;
    REFSIZEY = 0;
    Amplitude = X_m = S_x = Y_m = S_y = rho = 0;
    if (p.hasKey(FILTER_MOMENT_KEY))
    {
      moment_circle = p.getInt32Value(FILTER_MOMENT_KEY);
      apply_moment = true;
    }
#ifdef CERN_ROOT
    if (p.hasKey(GAUSS_FIT_KEY))
    {
      gauss_fit_level = 0;
      if (p.isCDataWrapperValue(GAUSS_FIT_KEY))
      {
        chaos::common::data::CDWUniquePtr gf = p.getCSDataValue(GAUSS_FIT_KEY);
        if (gf->hasKey("level"))
        {
          gauss_fit_level = gf->getInt32Value("level");
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
RTCameraFilter::~RTCameraFilter()
{
#ifdef CERN_ROOT
  if (apply_gauss_fit)
  {
    if (g2d)
    {
      delete g2d;
    }
    if (h)
    {
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
bool RTCameraFilter::setProp(const std::string &name, chaos::common::data::CDataWrapper p,
                             uint32_t size)
{

  if (name != "FILTER")
  {
    return false;
  }

  if (p.hasKey(FILTER_MOMENT_KEY) && p.isInt32Value(FILTER_MOMENT_KEY))
  {
    moment_circle = p.getInt32Value(FILTER_MOMENT_KEY);
    apply_moment = true;
  }
  else
  {
    apply_moment = true;
  }
  if (p.hasKey(FILTER_REMOVE_SOURCE_KEY) && p.isBoolValue(FILTER_REMOVE_SOURCE_KEY))
  {
    remove_src = p.getBoolValue(FILTER_REMOVE_SOURCE_KEY);
  }

  return true;
}

bool RTCameraFilter::setProp(const std::string &name, int32_t value,
                             uint32_t size)
{
  int ret;
  int32_t valuer;
  RTCameraFilterLDBG_ << "SET IPROP:" << name << " VALUE:" << value;
  SETINTPROP(name, REFOFFSETX, value);
  SETINTPROP(name, REFOFFSETY, value);
  SETINTPROP(name, REFSIZEX, value);
  SETINTPROP(name, REFSIZEY, value);

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

    addAttributeToDataSet("REFOFFSETX", "Reference Offset X",
                          chaos::DataType::TYPE_INT32, chaos::DataType::Input);
    addAttributeToDataSet("REFOFFSETY", "Reference Offset Y",
                          chaos::DataType::TYPE_INT32, chaos::DataType::Input);

    addAttributeToDataSet("REFSIZEX",
                          "Reference SIZEX",
                          chaos::DataType::TYPE_INT32, chaos::DataType::Input);
    addAttributeToDataSet("REFSIZEY",
                          "Reference SIZEY",
                          chaos::DataType::TYPE_INT32, chaos::DataType::Input);

    addHandlerOnInputAttributeName<::driver::sensor::camera::RTCameraFilter,
                                   int32_t>(
        this, &::driver::sensor::camera::RTCameraFilter::setProp, "REFOFFSETX");
    addHandlerOnInputAttributeName<::driver::sensor::camera::RTCameraFilter,
                                   int32_t>(
        this, &::driver::sensor::camera::RTCameraFilter::setProp, "REFOFFSETY");

    addHandlerOnInputAttributeName<::driver::sensor::camera::RTCameraFilter,
                                   int32_t>(
        this, &::driver::sensor::camera::RTCameraFilter::setProp, "REFSIZEX");
    addHandlerOnInputAttributeName<::driver::sensor::camera::RTCameraFilter,
                                   int32_t>(
        this, &::driver::sensor::camera::RTCameraFilter::setProp, "REFSIZEY");

    addStateVariable(StateVariableTypeAlarmCU, "momentx_out_of_set",
                     "moment X out of reference moment radius");
    addStateVariable(StateVariableTypeAlarmCU, "momenty_out_of_set",
                     "moment Y out of reference moment radius");
  }
  if (apply_gauss_fit)
  {
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
  if (apply_moment || apply_gauss_fit)
  {
    // convert image to grayscale
    cvtColor(image, gray, COLOR_BGR2GRAY);
  }
  if(apply_moment && !apply_gauss_fit){
    // convert grayscale to binary image
    threshold(gray, thr, 100, 255, THRESH_BINARY);
  }
  if (apply_gauss_fit)
  {
#ifdef CERN_ROOT
    if (h == NULL)
    {
      h = new TH2F("slm", " ", gray.cols, 0, gray.cols, gray.rows, 0, gray.rows);
      h->SetStats(0);
    }
    if (g2d == NULL)
    {
      g2d = new TF2("g2d", beamFunc, 0., gray.cols, 0., gray.rows, 6);
      //g2d->Draw("SAME");
      g2d->SetParNames("Amplitude", "X_m", "S_x", "Y_m", "S_y", "rho");
    }
    int xPixels = gray.cols;
    int yPixels = gray.rows;
    for (int row = 0; row < xPixels; ++row)
    {
      for (int col = 0; col < yPixels; ++col)
      {
        //float greyv = (255.0*1.0-gray.at<uchar>(row,col))/256.0;
        float greyv = (gray.at<uchar>(col, row)) / 256.0;
        //if (row<5 || row > xPixels-5 || col < 5 || col>yPixels-5) {
        //    grey = 0;
        //}
        //printf("%d(%d,%d) -> %f\n",index,row,col,grey);
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

    if (gauss_fit_level > 0)
    {
      g2d->SetParameters(par);

      RTCameraFilterLDBG_ << "Start Fitting params: A:" << par[0] << " Mx:" << par[1] << " Sx:" << par[2] << " My:" << par[3] << " Sy:" << par[4] << " rho:" << par[5];

      h->Fit("g2d", "N", "");
      Amplitude = g2d->GetParameter(0);
      X_m = g2d->GetParameter(1);
      S_x = g2d->GetParameter(2);
      Y_m = g2d->GetParameter(3);
      S_y = g2d->GetParameter(4);
      rho = g2d->GetParameter(5);
    }

    RTCameraFilterLDBG_ << "End Fitting C:" << X_m << "," << Y_m << " S:" << S_x << "," << S_y << " angle:" << rho;

    getAttributeCache()->setOutputAttributeValue("Amplitude", Amplitude);
    getAttributeCache()->setOutputAttributeValue("X_m", X_m);
    getAttributeCache()->setOutputAttributeValue("S_x", S_x);
    getAttributeCache()->setOutputAttributeValue("Y_m", Y_m);
    getAttributeCache()->setOutputAttributeValue("S_y", S_y);
    getAttributeCache()->setOutputAttributeValue("rho", rho);

#endif
  }
  if (apply_moment||apply_gauss_fit)
  {
    Point p;
    // find moments of the image
    if(apply_gauss_fit){
      p.x=X_m;
      p.y=Y_m;
    } else {
      Moments m = moments(thr, true);
      p.x=m.m10 / m.m00;
      p.y= m.m01 / m.m00;
    
    }
    getAttributeCache()->setOutputAttributeValue("MOMENTX", p.x);
    getAttributeCache()->setOutputAttributeValue("MOMENTY", p.y);

    if((S_x>0) && (S_y>0)){
      ellipse( image,
                 Point( X_m, Y_m),
                 Size( S_x, S_y ),
                 rho,
                 0,
                 360,
                 Scalar( 128, 0, 0 ),
                 1,
                 8 );

    } else if (moment_circle > 0)
    {
      circle(image, p, moment_circle, Scalar(128, 0, 0), -1);
    }
    if ((REFSIZEX > 0) && (REFSIZEY > 0))
    {
      double refx = REFOFFSETX + REFSIZEX * 1.0 / 2.0;
      double refy = REFOFFSETY + REFSIZEY * 1.0 / 2.0;
      int err = 0;
      setStateVariableSeverity(StateVariableTypeAlarmCU, "momentx_out_of_set",
                               chaos::common::alarm::MultiSeverityAlarmLevelClear);
      setStateVariableSeverity(StateVariableTypeAlarmCU, "momenty_out_of_set",
                               chaos::common::alarm::MultiSeverityAlarmLevelClear);

      if (!((REFOFFSETX <= p.x) && (p.x <= (REFOFFSETX + REFSIZEX))))
      {
        //
        setStateVariableSeverity(StateVariableTypeAlarmCU, "momentx_out_of_set",
                                 chaos::common::alarm::MultiSeverityAlarmLevelHigh);
        err++;
      }
      else if (!((REFOFFSETX <= (p.x - S_x)) && ((p.x + S_x) <= (REFOFFSETX + REFSIZEX))))
      {
        setStateVariableSeverity(StateVariableTypeAlarmCU, "momentx_out_of_set",
                                 chaos::common::alarm::MultiSeverityAlarmLevelWarning);
        err++;
      }
      if (!((REFOFFSETY <= p.y) && (p.y <= (REFOFFSETY + REFSIZEY))))
      {
        //
        setStateVariableSeverity(StateVariableTypeAlarmCU, "momenty_out_of_set",
                                 chaos::common::alarm::MultiSeverityAlarmLevelHigh);
        err++;
      }
      else if (!((REFOFFSETY <= (p.y - S_y)) && ((p.y + S_y) <= (REFOFFSETY + REFSIZEY))))
      {
        setStateVariableSeverity(StateVariableTypeAlarmCU, "momenty_out_of_set",
                                 chaos::common::alarm::MultiSeverityAlarmLevelWarning);
        err++;
      }

      cv::Rect r(REFOFFSETX, REFOFFSETY, REFSIZEX, REFSIZEY);
      if (err)
      {
        rectangle(image, r, Scalar(0, 0, 255), err);
      }
      else
      {
        rectangle(image, r, Scalar(0, 255, 0), 1);
      }
      double dist = sqrt(pow((p.x - refx), 2) + pow((p.y - refy), 2));
      getAttributeCache()->setOutputAttributeValue("MOMENTERR", dist);
    }
  }

  return 0;
}
