/*
 *	EpicsAreaDetector.h
 *	!CHAOS
 *	Created by Andrea Michelotti.
 *
 *    	Copyright 2022 INFN, National Institute of Nuclear Physics
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
#include "EpicsAreaDetector.h"
#include <chaos/cu_toolkit/driver_manager/driver/AbstractDriverPlugin.h>
//#include <driver/epics/driver/EpicsCAccessDriver.h>
#include <driver/epics/driver/EpicsPVAccessDriver.h>

#define MAX_RETRY 3
namespace cu_driver = chaos::cu::driver_manager::driver;
using namespace ::driver::sensor::camera;
#define EpicsAreaDetectorLAPP_ LAPP_ << "[EpicsAreaDetector] "
#define EpicsAreaDetectorLDBG LDBG_ << "[EpicsAreaDetector] "
#define EpicsAreaDetectorLERR LERR_ << "[EpicsAreaDetector] "

#define IMAGE_ARRAY "image1:ArrayData" //"Pva1:Image"
//#define IMAGE_ARRAY "Pva1:Image"
#define EpicsAreaDetectorLDBG_                                          \
  LDBG_ << "[EpicsAreaDetector (" << serial_dev << "," << friendly_name \
        << "):" << __FUNCTION__ << "] "
#define EpicsAreaDetectorLERR_                                          \
  LERR_ << "[EpicsAreaDetector (" << serial_dev << "," << friendly_name \
        << "):" << __PRETTY_FUNCTION__ << "] "
#define EpicsAreaDetectorLERR LERR_ << "[EpicsAreaDetector] "
// GET_PLUGIN_CLASS_DEFINITION
// we need only to define the driver because we don't are makeing a plugin
OPEN_CU_DRIVER_PLUGIN_CLASS_DEFINITION(EpicsAreaDetector, 1.0.0, ::driver::sensor::camera::EpicsAreaDetector)
REGISTER_CU_DRIVER_PLUGIN_CLASS_INIT_ATTRIBUTE(::driver::sensor::camera::EpicsAreaDetector, http_address / dnsname
                                               : port)
CLOSE_CU_DRIVER_PLUGIN_CLASS_DEFINITION
OPEN_REGISTER_PLUGIN
REGISTER_PLUGIN(::driver::sensor::camera::EpicsAreaDetector)
CLOSE_REGISTER_PLUGIN

#define GETINTNODE(x, camp, pub)                                           \
  {                                                                        \
    int32_t val;                                                           \
    LDBG_ << "GETTING INT PROP \"" << #x << "\" alias:" << pub;            \
    if (getNode(#x, camp, val, pub) == 0)                                  \
    {                                                                      \
    }                                                                      \
    else                                                                   \
    {                                                                      \
      EpicsAreaDetectorLERR << "cannot read ARAVIS node \"" << #x << "\""; \
    }                                                                      \
  }
#define GETDOUBLENODE(x, camp, pub)                                      \
  {                                                                      \
    double val;                                                          \
    LDBG_ << "GETTING double PROP \"" << #x << "\" alias:" << pub;       \
    if (getNode(#x, camp, val) == 0)                                     \
    {                                                                    \
    }                                                                    \
    else                                                                 \
    {                                                                    \
      EpicsAreaDetectorLERR << "cannot read ARAVIS double node \"" << #x \
                            << "\"";                                     \
    }                                                                    \
  }

#define GETINTVALUE(x, y, LOG)                                             \
  {                                                                        \
    int32_t val;                                                           \
    LOG << "GETTING INT PROP \"" << #x << "\" alias:" << y;                \
    if (getNode(#x, (&cam), val) == 0)                                     \
    {                                                                      \
      p->addInt32Value(y, (int32_t)val);                                   \
    }                                                                      \
    else                                                \
    {                                                                      \
      EpicsAreaDetectorLERR << "cannot read ARAVIS node \"" << #x << "\""; \
    }                                                                      \
  }
#define GETDOUBLEVALUE(x, y, LOG)                                          \
  {                                                                        \
    double val;                                                            \
    LOG << "GETTING DOUBLE PROP \"" << #x << "\" alias:" << y;             \
    if (getNode(#x, (&cam), val) == 0)                                     \
    {                                                                      \
      p->addDoubleValue(y, val);                                           \
    }                                                                      \
    else                                                                   \
    {                                                                      \
      EpicsAreaDetectorLERR << "cannot read ARAVIS node \"" << #x << "\""; \
    }                                                                      \
  }

#define GETINTPERCVALUE(x, y, LOG)                                  \
  {                                                                 \
    LOG << "GETTING INT PERCENT PROP \"" << #x << "\" alias:" << y; \
    float per;                                                      \
    if (getNodeInPercentage(#x, (&cam), per) == 0)                  \
    {                                                               \
      p->addDoubleValue(y, per);                                    \
    }                                                               \
  }

#define SETINODE(name, val, ret)                                           \
  if (setNode(name, (int32_t)val) == 0)                                    \
  {                                                                        \
    LDBG_ << "setting \"" << name << "\" = " << val;                       \
  }                                                                        \
  else                                                                     \
  {                                                                        \
    ret++;                                                                 \
    EpicsAreaDetectorLERR << "ERROR setting \"" << name << "\" = " << val; \
  }

#define STOP_GRABBING(camera)                   \
  if (camera == NULL)                           \
  {                                             \
    EpicsAreaDetectorLERR_ << "Invalid Camera"; \
    return -1;                                  \
  }                                             \
  if (!stopGrabbing)                            \
  {                                             \
    stopGrab();                                 \
    restore_grab = true;                        \
  }

#define RESTORE_GRABBING(camera) \
  if (restore_grab)              \
  {                              \
    startGrab(shots, NULL, fn);  \
  }

void EpicsAreaDetector::createProperties()
{
  std::vector<std::string> listPV = devicedriver->pvList();
  std::vector<std::string>::iterator i = listPV.begin();
  int retry = MAX_RETRY;
  while (i != listPV.end())
  {
    EpicsAreaDetectorLDBG_ << "retrieving information of " << *i; //<<" ="<<r->getJSONString();

    chaos::common::data::CDWUniquePtr r = devicedriver->readRecord(*i);
    if (r.get())
    {
      chaos::common::data::CDWUniquePtr conf = devicedriver->getPVConfig(*i);
      if (conf.get())
      {
        std::string cname;
        if (conf->hasKey(KEY_CNAME) && (r->hasKey(PROPERTY_VALUE_KEY)))
        {
          cname = conf->getStringValue(KEY_CNAME);
          EpicsAreaDetectorLDBG_ << "create PUBLIC property:" << *i << " CNAME:" << cname; //<<" ="<<r->getJSONString();
          createProperty(
              *i,
              [](AbstractDriver *thi, const std::string &name, const chaos::common::data::CDataWrapper &p)
                  -> chaos::common::data::CDWUniquePtr
              {
                // read handler
                return ((EpicsAreaDetector *)thi)->devicedriver->readRecord(name);
              },
              [](AbstractDriver *thi, const std::string &name, const chaos::common::data::CDataWrapper &p)
                  -> chaos::common::data::CDWUniquePtr
              {
                ((EpicsAreaDetector *)thi)->devicedriver->writeRecord(name, p);
                return chaos::common::data::CDWUniquePtr();
              },
              cname);

        } /* else {
           LDBG_ << "create  property:" << *i;  //<<" ="<<r->getJSONString();
         }*/
      }
      i++;
    }
    else
    {
      if (retry--)
      {
        devicedriver->waitChange(*i);
      }
      else
      {
        i++;
        retry = MAX_RETRY;
      }
    }
  }
}
void EpicsAreaDetector::driverInit(const chaos::common::data::CDataWrapper &json) throw(chaos::CException)
{
  EpicsAreaDetectorLDBG_ << "Configuration:" << json.getJSONString();
  // std::vector<std::string> pvlist ={"Pva1:Image"};
  // std::vector<std::string> pvlist ={IMAGE_ARRAY,"cam1:DataType_RBV","cam1:ColorMode_RBV"};
  chaos::common::data::CDWUniquePtr newconf = json.clone();
  //::driver::epics::common::EpicsGenericDriver::addPVListConfig(*(newconf.get()), pvlist);

  std::map<std::string, std::string> pvprprop = {
      {"cam1:Acquire", ""},
      {"cam1:SizeX_RBV", ""},
      {"cam1:SizeY_RBV", ""},
      {"cam1:ArraySizeX_RBV", ""},
      {"cam1:ArraySizeY_RBV", ""},
      {"cam1:MaxSizeX_RBV", ""},
      {"cam1:MaxSizeY_RBV", ""},
      {"cam1:SizeX", ""},
      {"cam1:SizeY", ""},
      {"cam1:MinX_RBV", ""},
      {"cam1:MinY_RBV", ""},
      {"cam1:MinX", ""},
      {"cam1:MinY", ""},
      {"cam1:TriggerMode", ""},
      {"cam1:TriggerMode_RBV", ""},
      {"cam1:DataType_RBV", ""},
      {"cam1:ColorMode_RBV", ""},
      {"cam1:AcquireTime", ""},
      {"cam1:AcquireTime_RBV", ""},
      {"cam1:AcquirePeriod", ""},
      {"cam1:AcquirePeriod_RBV", ""},
      {"cam1:Gain", ""},
      {"cam1:Gain_RBV", ""},
      {"cam1:Manufacturer_RBV", ""},
      {"cam1:Model_RBV", ""},
      {"cam1:SerialNumber_RBV", ""},
      {"cam1:FirmwareVersion_RBV", ""},
      {"cam1:DriverVersion_RBV", ""}

  };
  std::string image_array=IMAGE_ARRAY;
  if(json.hasKey("PVIMAGE")){
    image_array=json.getStringValue("PVIMAGE");
  }
  pvprprop[image_array]="";
  EpicsAreaDetectorLDBG_ << "Using \"" << image_array << "\" as PV image array";
  ::driver::epics::common::EpicsGenericDriver::addPVListConfig(*(newconf.get()), pvprprop);

  devicedriver = new ::driver::epics::common::EpicsPVAccessDriver(*(newconf.get()));

  createProperty(
      "SizeX",
      [](AbstractDriver *thi, const std::string &name,
         const chaos::common::data::CDataWrapper &p)
          -> chaos::common::data::CDWUniquePtr
      {
        chaos::common::data::CDWUniquePtr ret(new chaos::common::data::CDataWrapper());

        EpicsAreaDetector *t = (EpicsAreaDetector *)thi;
        int32_t val = 0;
        t->devicedriver->read("cam1:SizeX_RBV", val);
        if (val)
        {
          ret->addInt32Value(PROPERTY_VALUE_KEY, val);
          t->sizex = val;
        }
        t->devicedriver->read("cam1:MaxSizeX_RBV", t->cam_sizex);
        ret->append(PROPERTY_VALUE_MIN_KEY, 0);
        ret->append(PROPERTY_VALUE_MAX_KEY, t->cam_sizex);
        return ret;
      },
      [](AbstractDriver *thi, const std::string &name,
         const chaos::common::data::CDataWrapper &p)
          -> chaos::common::data::CDWUniquePtr
      {
        EpicsAreaDetector *t = (EpicsAreaDetector *)thi;
        int32_t val = p.getInt32Value(PROPERTY_VALUE_KEY);
        if (val > t->cam_sizex)
          val = t->cam_sizex;
        t->devicedriver->write("cam1:SizeX", val);
        t->sizex = val;

        return chaos::common::data::CDWUniquePtr();
      },
      "WIDTH");

  createProperty(
      "SizeY",
      [](AbstractDriver *thi, const std::string &name,
         const chaos::common::data::CDataWrapper &p)
          -> chaos::common::data::CDWUniquePtr
      {
        chaos::common::data::CDWUniquePtr ret(new chaos::common::data::CDataWrapper());

        EpicsAreaDetector *t = (EpicsAreaDetector *)thi;
        int32_t val = 0;
        t->devicedriver->read("cam1:SizeY_RBV", val);
        if (val)
        {
          ret->addInt32Value(PROPERTY_VALUE_KEY, val);
          t->sizey = val;
        }
        t->devicedriver->read("cam1:MaxSizeY_RBV", t->cam_sizey);
        ret->append(PROPERTY_VALUE_MIN_KEY, 0);
        ret->append(PROPERTY_VALUE_MAX_KEY, t->cam_sizey);
        return ret;
      },
      [](AbstractDriver *thi, const std::string &name,
         const chaos::common::data::CDataWrapper &p)
          -> chaos::common::data::CDWUniquePtr
      {
        EpicsAreaDetector *t = (EpicsAreaDetector *)thi;
        int32_t val = p.getInt32Value(PROPERTY_VALUE_KEY);
        if (val > t->cam_sizey)
          val = t->cam_sizey;
        t->devicedriver->write("cam1:SizeY", val);
        t->sizey = val;

        return chaos::common::data::CDWUniquePtr();
      },
      "HEIGHT");
  createProperty(
      "MinX",
      [](AbstractDriver *thi, const std::string &name,
         const chaos::common::data::CDataWrapper &p)
          -> chaos::common::data::CDWUniquePtr
      {
        chaos::common::data::CDWUniquePtr ret(new chaos::common::data::CDataWrapper());

        EpicsAreaDetector *t = (EpicsAreaDetector *)thi;
        int32_t val = 0;
        t->devicedriver->read("cam1:MinX_RBV", val);
        t->offx = val;
        ret->addInt32Value(PROPERTY_VALUE_KEY, val);

        return ret;
      },
      [](AbstractDriver *thi, const std::string &name,
         const chaos::common::data::CDataWrapper &p)
          -> chaos::common::data::CDWUniquePtr
      {
        EpicsAreaDetector *t = (EpicsAreaDetector *)thi;
        int32_t val = p.getInt32Value(PROPERTY_VALUE_KEY);
        t->devicedriver->write("cam1:MinX", val);
        t->offx = val;

        return chaos::common::data::CDWUniquePtr();
      },
      "OFFSETX");
  createProperty(
      "MinY",
      [](AbstractDriver *thi, const std::string &name,
         const chaos::common::data::CDataWrapper &p)
          -> chaos::common::data::CDWUniquePtr
      {
        chaos::common::data::CDWUniquePtr ret(new chaos::common::data::CDataWrapper());

        EpicsAreaDetector *t = (EpicsAreaDetector *)thi;
        int32_t val = 0;
        t->devicedriver->read("cam1:MinY_RBV", val);
        ret->addInt32Value(PROPERTY_VALUE_KEY, val);
        t->offy = val;

        return ret;
      },
      [](AbstractDriver *thi, const std::string &name,
         const chaos::common::data::CDataWrapper &p)
          -> chaos::common::data::CDWUniquePtr
      {
        EpicsAreaDetector *t = (EpicsAreaDetector *)thi;
        int32_t val = p.getInt32Value(PROPERTY_VALUE_KEY);
        t->devicedriver->write("cam1:MinY", val);
        t->offy = val;

        return chaos::common::data::CDWUniquePtr();
      },
      "OFFSETY");
  createProperty(
      "AcquireTime",
      [](AbstractDriver *thi, const std::string &name,
         const chaos::common::data::CDataWrapper &p)
          -> chaos::common::data::CDWUniquePtr
      {
        std::string val;
        chaos::common::data::CDWUniquePtr ret(new chaos::common::data::CDataWrapper());

        EpicsAreaDetector *t = (EpicsAreaDetector *)thi;
        double shutter = 0;
        t->devicedriver->read("cam1:AcquireTime_RBV", shutter);
        if (shutter)
        {
          ret->addDoubleValue(PROPERTY_VALUE_KEY, 1000000 * shutter);
          ret->addDoubleValue("raw", shutter);
        }

        return ret;
      },
      [](AbstractDriver *thi, const std::string &name,
         const chaos::common::data::CDataWrapper &p)
          -> chaos::common::data::CDWUniquePtr
      {
        EpicsAreaDetector *t = (EpicsAreaDetector *)thi;
        double val = p.getDoubleValue(PROPERTY_VALUE_KEY);
        t->devicedriver->write("cam1:AcquireTime", val / 1000000.0);
        return chaos::common::data::CDWUniquePtr();
      },
      SHUTTER_KEY);

  createProperty(
      "AcquirePeriod",
      [](AbstractDriver *thi, const std::string &name,
         const chaos::common::data::CDataWrapper &p)
          -> chaos::common::data::CDWUniquePtr
      {
        chaos::common::data::CDWUniquePtr ret(new chaos::common::data::CDataWrapper());

        EpicsAreaDetector *t = (EpicsAreaDetector *)thi;
        double val = 0;
        t->devicedriver->read("cam1:AcquirePeriod_RBV", val);
        if (val)
        {
          ret->addDoubleValue(PROPERTY_VALUE_KEY, 1.0 / val);
          ret->addDoubleValue("raw", val);
        }

        return ret;
      },
      [](AbstractDriver *thi, const std::string &name,
         const chaos::common::data::CDataWrapper &p)
          -> chaos::common::data::CDWUniquePtr
      {
        EpicsAreaDetector *t = (EpicsAreaDetector *)thi;
        double val = p.getDoubleValue(PROPERTY_VALUE_KEY);
        t->devicedriver->write("cam1:AcquirePeriod", 1 / val);
        return chaos::common::data::CDWUniquePtr();
      },
      "FRAMERATE");

  createProperty(
      "Gain",
      [](AbstractDriver *thi, const std::string &name,
         const chaos::common::data::CDataWrapper &p)
          -> chaos::common::data::CDWUniquePtr
      {
        chaos::common::data::CDWUniquePtr ret(new chaos::common::data::CDataWrapper());

        EpicsAreaDetector *t = (EpicsAreaDetector *)thi;
        double val = 0;
        t->devicedriver->read("cam1:Gain_RBV", val);
        if (val)
        {
          ret->addDoubleValue(PROPERTY_VALUE_KEY, val);
        }

        return ret;
      },
      [](AbstractDriver *thi, const std::string &name,
         const chaos::common::data::CDataWrapper &p)
          -> chaos::common::data::CDWUniquePtr
      {
        EpicsAreaDetector *t = (EpicsAreaDetector *)thi;
        double val = p.getDoubleValue(PROPERTY_VALUE_KEY);
        t->devicedriver->write("cam1:Gain", val);
        return chaos::common::data::CDWUniquePtr();
      },
      GAIN_KEY);

  createProperty(
      "PixelFormat", "", FRAMEBUFFER_ENCODING_KEY,
      [](AbstractDriver *thi, const std::string &name,
         const chaos::common::data::CDataWrapper &p)
          -> chaos::common::data::CDWUniquePtr
      {
        std::string val;
        chaos::common::data::CDWUniquePtr ret(new chaos::common::data::CDataWrapper());

        EpicsAreaDetector *t = (EpicsAreaDetector *)thi;
        t->devicedriver->read("cam1:ColorMode_RBV", t->colormode);
        t->devicedriver->read("cam1:DataType_RBV", t->datatype);

        switch (t->colormode)
        {

          /*
          DataTYpe:
           [ 0] Int8
            [ 1] UInt8
            [ 2] Int16
            [ 3] UInt16
            [ 4] Int32
            [ 5] UInt32
            [ 6] Int64
            [ 7] UInt64
            [ 8] Float32
            [ 9] Float64
          Color Mode:
           [ 0] Mono
            [ 1] Bayer
            [ 2] RGB1
            [ 3] RGB2
            [ 4] RGB3
            [ 5] YUV444
            [ 6] YUV422
            [ 7] YUV421
          */
         case 0: //MONO
          if (t->datatype == 0)
          {
            ret->addStringValue(PROPERTY_VALUE_KEY, std::string("CV_8SC1"));
          }
          else if (t->datatype == 1)
          {
            ret->addStringValue(PROPERTY_VALUE_KEY, std::string("CV_8UC1"));
          }
          else if (t->datatype == 2)
          {
            ret->addStringValue(PROPERTY_VALUE_KEY, std::string("CV_16SC1"));
          }
          else if (t->datatype == 3)
          {
            ret->addStringValue(PROPERTY_VALUE_KEY, std::string("CV_16UC1"));
          }
          else if (t->datatype == 4)
          {
            ret->addStringValue(PROPERTY_VALUE_KEY, std::string("CV_32SC1"));
          }
          else if (t->datatype == 5)
          {
            ret->addStringValue(PROPERTY_VALUE_KEY, std::string("CV_32UC1"));
          } else {
            std::stringstream ss;
                      ss<<"not handled type:"<<t->datatype<<" color mode:"<<t->colormode;
                      ret->addStringValue(PROPERTY_VALUE_KEY, ss.str());
          }
          break;
        case 1: // BAYER
          if (t->datatype == 0)
          {
            ret->addStringValue(PROPERTY_VALUE_KEY, std::string("BAYERBG8"));
          }
          else if (t->datatype == 1)
          {
            ret->addStringValue(PROPERTY_VALUE_KEY, std::string("BAYERBG8"));
          }
          else if (t->datatype == 2)
          {
            ret->addStringValue(PROPERTY_VALUE_KEY, std::string("BAYERBG16"));
          }
          else if (t->datatype == 3)
          {
            ret->addStringValue(PROPERTY_VALUE_KEY, std::string("BAYERBG16"));
          }else {
                        std::stringstream ss;

                      ss<<"not handled type:"<<t->datatype<<" color mode:"<<t->colormode;
                      ret->addStringValue(PROPERTY_VALUE_KEY, ss.str());
          }
          break;
        case 2: // RGB1
        case 3:
        case 4:
          ret->addStringValue(PROPERTY_VALUE_KEY, std::string("CV_8UC3"));
          break;
        case 5: // YUV444
          ret->addStringValue(PROPERTY_VALUE_KEY, std::string("YUV444packed"));

          break;
        case 6: // YUV422
          ret->addStringValue(PROPERTY_VALUE_KEY, std::string("YUV422packed"));
          break;
        case 7: // YUV421
          ret->addStringValue(PROPERTY_VALUE_KEY, std::string("YUV422packed"));

          break;
        default:{
          std::stringstream ss;
          ss<<" not handled case color mode:"<<t->colormode <<" data type:"<<t->datatype;
          ret->addStringValue(PROPERTY_VALUE_KEY, ss.str());
        }

        }
        return ret;

      },
      [](AbstractDriver *thi, const std::string &name,
         const chaos::common::data::CDataWrapper &p)
          -> chaos::common::data::CDWUniquePtr
      {
        EpicsAreaDetector *t = (EpicsAreaDetector *)thi;

        return chaos::common::data::CDWUniquePtr();
      });
  createProperty(
      "TriggerMode",
      [](AbstractDriver *thi, const std::string &name,
         const chaos::common::data::CDataWrapper &p)
          -> chaos::common::data::CDWUniquePtr
      {
        EpicsAreaDetector *t = (EpicsAreaDetector *)thi;
        std::string ton;
        chaos::common::data::CDWUniquePtr ret(new chaos::common::data::CDataWrapper());
        int16_t trigger_enable;
        t->devicedriver->read("cam1:TriggerMode_RBV", trigger_enable);
        if (!trigger_enable)
        {
          t->tmode = CAMERA_TRIGGER_CONTINOUS;
          ret->addInt32Value(PROPERTY_VALUE_KEY, t->tmode);
        }
        else
        {
          t->tmode = CAMERA_TRIGGER_HW_HI;

          ret->addInt32Value(PROPERTY_VALUE_KEY, t->tmode);
        }
        return ret;
      },
      [](AbstractDriver *thi, const std::string &name,
         const chaos::common::data::CDataWrapper &p)
          -> chaos::common::data::CDWUniquePtr
      {
            EpicsAreaDetector *t = (EpicsAreaDetector *)thi;
          LDBG_ << name << " Trigger Mode WRITE" << p.getJSONString();

          

            int32_t vale = p.getInt32Value(PROPERTY_VALUE_KEY);
            if(vale==CAMERA_TRIGGER_CONTINOUS){
                t->devicedriver->write("cam1:TriggerMode",0);
            } else {
                t->devicedriver->write("cam1:TriggerMode",1);

            } 
            t->tmode = (driver::sensor::camera::TriggerModes)vale;

                    return chaos::common::data::CDWUniquePtr();

            },
      "TRIGGER_MODE");

  createProperties();
}
void EpicsAreaDetector::driverDeinit()
{
  if (devicedriver)
  {
    EpicsAreaDetectorLDBG_ << "Destroy";

    delete devicedriver;
    devicedriver = NULL;
  }
}

void EpicsAreaDetector::driverInit(const char *initParameter) throw(chaos::CException)
{
  if (initParameter != NULL)
  {
    chaos::common::data::CDataWrapper cw;
    cw.setSerializedJsonData(initParameter);
    driverInit(cw);
  }
  throw chaos::CException(-1, "invalid configuration", __PRETTY_FUNCTION__);
}

namespace driver
{
  namespace sensor
  {
    namespace camera
    {

      int EpicsAreaDetector::setNode(const std::string &node_name, bool val)
      {

        return 0;
      }
      int EpicsAreaDetector::setNode(const std::string &node_name, std::string val)
      {

        return 0;
      }

      int EpicsAreaDetector::setNode(const std::string &node_name, int32_t val)
      {

        return 0;
      }

      int EpicsAreaDetector::setNode(const std::string &node_name, double val)
      {

        return 0;
      }

      int EpicsAreaDetector::getNode(const std::string &node_name, std::string &val)
      {

        return 0;
      }

      int EpicsAreaDetector::getNode(const std::string &node_name, bool &val)
      {

        return 0;
      }

      int EpicsAreaDetector::getNode(const std::string &node_name, double &percent,
                                     double &max, double &min, double &inc)
      {

        return 0;
      }
      int EpicsAreaDetector::getNode(const std::string &node_name, int32_t &percent,
                                     int32_t &max, int32_t &min, int32_t &inc)
      {

        return 0;
      }

    } // namespace camera
  }   // namespace sensor
} // namespace driver

#define CREATE_VALUE_PROP(n, pub, type)                                      \
  createProperty(                                                            \
      n,                                                                     \
      [](AbstractDriver *thi, const std::string &name,                       \
         const chaos::common::data::CDataWrapper &p)                         \
          -> chaos::common::data::CDWUniquePtr {                             \
        type val, max, min, inc;                                             \
        if (((EpicsAreaDetector *)thi)->getNode(name, val, max, min, inc) == \
            0)                                                               \
        {                                                                    \
          chaos::common::data::CDWUniquePtr ret(                             \
              new chaos::common::data::CDataWrapper());                      \
          ret->append(PROPERTY_VALUE_KEY, val);                              \
          ret->append(PROPERTY_VALUE_MIN_KEY, min);                          \
          ret->append(PROPERTY_VALUE_MAX_KEY, max);                          \
          ret->append(PROPERTY_VALUE_INC_KEY, inc);                          \
          return ret;                                                        \
        }                                                                    \
        EpicsAreaDetectorLERR << " cannot get " #type << " " << name;        \
        return chaos::common::data::CDWUniquePtr();                          \
      },                                                                     \
      [](AbstractDriver *thi, const std::string &name,                       \
         const chaos::common::data::CDataWrapper &p)                         \
          -> chaos::common::data::CDWUniquePtr {                             \
        type val = p.getValue<type>(PROPERTY_VALUE_KEY);                     \
        if (((EpicsAreaDetector *)thi)->setNode(name, val) != 0)             \
        {                                                                    \
          EpicsAreaDetectorLERR << " cannot set " #type << " " << name       \
                                << " to:" << val;                            \
          return chaos::common::data::CDWUniquePtr();                        \
        }                                                                    \
        return p.clone();                                                    \
      },                                                                     \
      pub);

#define CREATE_PROP(n, pub, typ)                                  \
  createProperty(                                                 \
      n,                                                          \
      [](AbstractDriver *thi, const std::string &name,            \
         const chaos::common::data::CDataWrapper &p)              \
          -> chaos::common::data::CDWUniquePtr {                  \
        typ _val;                                                 \
        if (((EpicsAreaDetector *)thi)->getNode(name, _val) == 0) \
        {                                                         \
          chaos::common::data::CDWUniquePtr ret(                  \
              new chaos::common::data::CDataWrapper());           \
          ret->append(PROPERTY_VALUE_KEY, _val);                  \
          return ret;                                             \
        }                                                         \
        EpicsAreaDetectorLERR << " cannot get " #typ << " "       \
                              << name;                            \
        return chaos::common::data::CDWUniquePtr();               \
      },                                                          \
      [](AbstractDriver *thi, const std::string &name,            \
         const chaos::common::data::CDataWrapper &p)              \
          -> chaos::common::data::CDWUniquePtr {                  \
        typ _val = p.getValue<typ>(PROPERTY_VALUE_KEY);           \
        if (((EpicsAreaDetector *)thi)->setNode(name, _val) != 0) \
        {                                                         \
          EpicsAreaDetectorLERR << " cannot set " #typ << " "     \
                                << name << " to:" << _val;        \
          return chaos::common::data::CDWUniquePtr();             \
        }                                                         \
        return p.clone();                                         \
      },                                                          \
      pub);

static int cv2ARAVIS(const std::string &fmt)
{
  if (fmt == "CV_8UC1")
    return 0; // Pylon::EPixelType::PixelType_Mono8;
  if (fmt == "CV_8SC1")
    return 1; // Pylon::EPixelType::PixelType_Mono8signed;
  if (fmt == "CV_16UC1")
    return 2; // Pylon::EPixelType::PixelType_Mono16;
  if (fmt == "CV_8UC3")
    return 3; // Pylon::EPixelType::PixelType_RGB8packed;
  return 4;   /*Pylon::EPixelType::PixelType_Mono8:*/
}
static std::string ARAVIS2cv(int fmt)
{
  return "CV_8UC1";
}

int EpicsAreaDetector::initializeCamera(
    const chaos::common::data::CDataWrapper &json)
{
  int found = 0;
  EpicsAreaDetectorLDBG_ << "getting  camera informations ..";

  return 0;
}
/*
EpicsAreaDetector::EpicsAreaDetector():camerap(NULL),shots(0),framebuf(NULL),fn(NULL),props(NULL),tmode(CAMERA_TRIGGER_CONTINOUS),gstrategy(CAMERA_LATEST_ONLY){

    EpicsAreaDetectorLDBG_<<  "Created ARAVIS Driver";
    props=new chaos::common::data::CDataWrapper();


}
*/

EpicsAreaDetector::EpicsAreaDetector() : devicedriver(NULL), sizex(0), sizey(0), offx(0), offy(0), cam_sizex(0), cam_sizey(0), size_image(0)
{
  EpicsAreaDetectorLDBG_ << "Created Epics AreaDetector driver ";

  stopGrabbing = true;
  restore_grab = false;
}

// default descrutcor
EpicsAreaDetector::~EpicsAreaDetector()
{
  delete devicedriver;
}

int EpicsAreaDetector::cameraToProps(chaos::common::data::CDataWrapper *p)
{
  int32_t ivalue;
  if (p == NULL)
  {
    EpicsAreaDetectorLERR_ << "Invalid Parameter";
    return -1;
  }
  EpicsAreaDetectorLDBG_ << " synchronize all properties..";

  syncRead(); // synchronize with real values
  appendPubPropertiesTo(*p);

  return 0;
}

int EpicsAreaDetector::propsToCamera(chaos::common::data::CDataWrapper *p)
{
  // Get the camera control object.
  int ret = 0;
  if (p == NULL)
  {
    EpicsAreaDetectorLDBG_ << "Invalid Parameter";
    return -1;
  }
  EpicsAreaDetectorLDBG_ << "setting props: " << p->getCompliantJSONString();

  // Get the parameters for setting the image area of interest (Image AOI).
  setProperties(*p, true);

  return ret;
}

int EpicsAreaDetector::cameraInit(void *buffer, uint32_t sizeb)
{
  EpicsAreaDetectorLDBG_ << "Deprecated Initialization";
  // simple initialization

  return 0;
}

int EpicsAreaDetector::cameraDeinit()
{
  EpicsAreaDetectorLDBG_ << "deinit";

  return 0;
}

int EpicsAreaDetector::startGrab(uint32_t _shots, void *_framebuf,
                                 cameraGrabCallBack _fn)
{
  EpicsAreaDetectorLDBG_ << "Start Grabbing";
  int ret = devicedriver->read("cam1:SizeX_RBV", sizex);
  if (ret <= 0)
  {
    EpicsAreaDetectorLERR_ << "cannot get sizex " << ret;
    return ret;
  }
  ret = devicedriver->read("cam1:SizeY_RBV", sizey);
  if (ret <= 0)
  {
    EpicsAreaDetectorLERR_ << "cannot get sizey " << ret;
    return ret;
  }
  ret = devicedriver->read("cam1:DataType_RBV", datatype);
  if (ret <= 0)
  {
    EpicsAreaDetectorLERR_ << "cannot get data type " << ret;
    return ret;
  }
  ret = devicedriver->read("cam1:ColorMode_RBV", colormode);
  if (ret <= 0)
  {
    EpicsAreaDetectorLERR_ << "cannot get ColorMode " << ret;
    return ret;
  }
  ret = devicedriver->read("cam1:MinX_RBV", offx);
  if (ret <= 0)
  {
    EpicsAreaDetectorLERR_ << "cannot get MinX_RBV " << ret;
    return ret;
  }
  ret = devicedriver->read("cam1:MinY_RBV", offy);
  if (ret <= 0)
  {
    EpicsAreaDetectorLERR_ << "cannot get MinY_RBV " << ret;
    return ret;
  }
  devicedriver->read("cam1:MaxSizeX_RBV", cam_sizex);
  devicedriver->read("cam1:MaxSizeY_RBV", cam_sizey);
  devicedriver->write("cam1:Acquire",1);
  EpicsAreaDetectorLDBG_ << "Start grabbing " << sizex << " x " << sizey << " off:" << offx << "," << offy << " cam " << cam_sizex << " x " << cam_sizey;

  stopGrabbing = false;
  return 0;
}
int EpicsAreaDetector::waitGrab(camera_buf_t **img, uint32_t timeout_ms)
{
  if (stopGrabbing)
  {
    EpicsAreaDetectorLERR_ << "Grabbing is stopped ";
    return CAMERA_GRAB_STOP;
  }
  size_t size = 0;
  *img = new camera_buf_t(sizex * sizey,
                          sizex,
                          sizey,
                          offx,
                          offy);
 // ret = devicedriver->readArray(IMAGE_ARRAY, (*img)->buf, sizex * sizey, 2);
  ::driver::epics::common::epics_ts_t ts;
  ::driver::epics::common::epics_stats_t stats;
  int ret = devicedriver->read(IMAGE_ARRAY,(void*) (*img)->buf, sizex * sizey,timeout_ms,&ts,&stats);
  (*img)->ts = ::driver::epics::common::epicsTS2US(ts);

  if (ret <= 0)
  {
    delete *img;
    *img = NULL;
    EpicsAreaDetectorLERR_ << "Error reading array image PV:\"" << IMAGE_ARRAY << "\" ret:" << ret;
    return ret;
  }
  if(stats.sev!=0){
    setLastError(stats.msg);
    EpicsAreaDetectorLERR_ << "Severity "<<stats.sev<<" msg:\"" << stats.msg << "\" ret:" << ret;

    return stats.sev+chaos::ErrorCode::EC_EPICS_SEVERITY;
  } 
  return ret;
}

int EpicsAreaDetector::waitGrab(uint32_t timeout_ms)
{
  return waitGrab((camera_buf_t **)NULL, timeout_ms);
}
int EpicsAreaDetector::stopGrab()
{
  EpicsAreaDetectorLDBG_ << "Stop  Grabbing, unblock waiting";
  devicedriver->write("cam1:Acquire",0);

  stopGrabbing = true;

  return 0;
}

int EpicsAreaDetector::setImageProperties(int32_t width, int32_t height,
                                          int32_t opencvImageType)
{
  // props->setValue("WIDTH", width);
  // props->setValue("HEIGHT", width);
  int ret = 0;
  //  if (camerap)
  {
    // return propsToCamera(*camerap, props.get());
    ret = (setPropertyValue("WIDTH", width).get()) ? 0 : -1;
    ret += (setPropertyValue("HEIGHT", height).get()) ? 0 : -1;
  }
  return ret;
}

int EpicsAreaDetector::getImageProperties(int32_t &width, int32_t &height,
                                          int32_t &opencvImageType)
{
  ChaosUniquePtr<chaos::common::data::CDataWrapper> cw(
      new chaos::common::data::CDataWrapper());
  int ret = -1;
  cameraToProps(cw.get());
  if (cw->hasKey("WIDTH"))
  {
    width = cw->getInt32Value("WIDTH");
    ret = 0;
  }

  if (cw->hasKey("HEIGHT"))
  {
    height = cw->getInt32Value("HEIGHT");
    ret++;
  }

  return (ret > 0) ? 0 : ret;
}

int EpicsAreaDetector::setCameraProperty(const std::string &propname,
                                         int32_t val)
{
  ChaosUniquePtr<chaos::common::data::CDataWrapper> cw(
      new chaos::common::data::CDataWrapper());
  EpicsAreaDetectorLDBG_ << "Setting Int \"" << propname
                         << "\"=" << (int32_t)val;

  cw->addInt32Value(propname, val);
  int ret = propsToCamera(cw.get());
  return ret;
}
int EpicsAreaDetector::setCameraProperty(const std::string &propname,
                                         double val)
{
  ChaosUniquePtr<chaos::common::data::CDataWrapper> cw(
      new chaos::common::data::CDataWrapper());
  EpicsAreaDetectorLDBG_ << "Setting double \"" << propname << "\"=" << val;

  cw->addDoubleValue(propname, val);
  int ret = propsToCamera(cw.get());
  return ret;
}

int EpicsAreaDetector::getCameraProperty(const std::string &propname,
                                         int32_t &val)
{
  ChaosUniquePtr<chaos::common::data::CDataWrapper> cw(
      new chaos::common::data::CDataWrapper());

  cameraToProps(cw.get());

  if (cw->hasKey(propname))
  {
    val = cw->getInt32Value(propname);
    EpicsAreaDetectorLDBG_ << "Property Int \"" << propname << "\"=" << val;

    return 0;
  }
  return -1;
}

int EpicsAreaDetector::getCameraProperty(const std::string &propname,
                                         double &val)
{
  ChaosUniquePtr<chaos::common::data::CDataWrapper> cw(
      new chaos::common::data::CDataWrapper());

  cameraToProps(cw.get());

  if (cw->hasKey(propname))
  {
    val = cw->getDoubleValue(propname);
    EpicsAreaDetectorLDBG_ << "Property Double \"" << propname << "\"=" << val;

    return 0;
  }
  return -1;
}

int EpicsAreaDetector::getCameraProperties(
    chaos::common::data::CDataWrapper &proplist)
{
  int ret = cameraToProps(&proplist);
  EpicsAreaDetectorLDBG_ << "PROPERTIES:" << proplist.getCompliantJSONString()
                         << " ret:" << ret;

  return ret;
}

int EpicsAreaDetector::cameraRoi(int sizex,int sizey,int x, int y){
  EpicsAreaDetectorLDBG_ << "Performing ROI" << sizex << "x" << sizey << "("<< x << "," << y << ")";

  setPropertyValue("WIDTH", sizex);
  setPropertyValue("HEIGHT", sizey);
  setPropertyValue("OFFSETX", x);
  setPropertyValue("OFFSETY", y);
  return 0;
}
