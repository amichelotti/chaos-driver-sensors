/*
 *	RTCameraBase.h
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
#ifndef ChaosRTControlUnit_RTCameraBase_h
#define ChaosRTControlUnit_RTCameraBase_h
#include  <chaos/common/thread/TLockFreeQueue.h> 
#include <common/misc/data/core/SharedMem.h>
#include <chaos/cu_toolkit/control_manager/RTAbstractControlUnit.h>

#define DEFAULT_RESOLUTION 640*480*3
#define CAMERA_FRAME_BUFFERING 2

namespace cv {class Mat;}
namespace driver{
    
    namespace sensor{
         namespace camera{
	   class CameraDriverInterface;
     class camera_buf_t;
class RTCameraBase : public chaos::cu::control_manager::RTAbstractControlUnit {
	PUBLISHABLE_CONTROL_UNIT_INTERFACE(RTCameraBase);


public:
    /*!
     Construct a new CU with full constructor
     */
    RTCameraBase(const std::string& _control_unit_id, const std::string& _control_unit_param, const ControlUnitDriverList& _control_unit_drivers,const int buffering=CAMERA_FRAME_BUFFERING);
    /*!
     Destructor a new CU
     */
    ~RTCameraBase();
        void cameraGrabCallBack(const void*buf,uint32_t blen,uint32_t width,uint32_t heigth, uint32_t error);

protected:
        const int buffering;
        cv::Mat* subImage;
        bool applyCalib,performCalib,applyReference;
        int refenceThick,refenceR,refenceG,refenceB;
        

        std::string calibimage;
        ChaosUniquePtr<::common::misc::data::SharedMem> shared_mem;
        int32_t *sizex,*sizey,*offsetx,*offsety,*osizex,*osizey,*ooffsetx,*ooffsety;
        // if >0 then this is the camera window created, each grab should fit this size.
        int32_t imagesizex,imagesizey;
        int bpp;
        bool apply_resize;
        int32_t mode,*omode;
        uint8_t* framebuf;
        uint32_t framebuf_encoding;
        std::string framebuf_encoding_s;
        bool*pacquire,*ptrigger,*ppulse;
        const int32_t *refsx,*refsy,*refrho;
        const int32_t *refx,*refy;
        //double ZOOMX,ZOOMY;
        chaos::common::data::CDataWrapper filters;
        
        typedef struct encoded {
            std::vector<unsigned char>* img;
            int sizex,sizey,offsetx,offsety;
            encoded():img(NULL){};
        } encoded_t;

        char encoding[16];
     //   chaos::common::data::CDataWrapper camera_props;
       // uint8_t* camera_out;
      //  buf_t framebuf_out[CAMERA_FRAME_BUFFERING]; //capture stage
        std::vector<int> encode_params;

        void captureThread();
        bool stopCapture,stopEncoding;
        boost::thread capture_th,encode_th;
      //  std::vector<unsigned char> encbuf[CAMERA_FRAME_BUFFERING];//encode stage
        chaos::common::thread::TLockFreeQueue<::driver::sensor::camera::camera_buf_t*,CAMERA_FRAME_BUFFERING> captureImg;
        chaos::common::thread::TLockFreeQueue<encoded_t,CAMERA_FRAME_BUFFERING> encodedImg;

        //boost::condition_variable wait_capture,wait_encode,full_capture,full_encode;

        //boost::lockfree::queue<::driver::sensor::camera::camera_buf_t*, boost::lockfree::fixed_sized<true> > captureImg;
        //boost::lockfree::queue<encoded_t, boost::lockfree::fixed_sized<true> > encodedImg;
        //boost::mutex mutex_io,mutex_encode;

        uint32_t hw_trigger_timeout_us,sw_trigger_timeout_us,trigger_timeout; // 0 =wait indefinitively
        
        uint64_t encode_time,capture_time,network_time,counter_capture,counter_encode;
        void encodeThread();
        int bufinuse;
        int32_t* enc_frame_rate,*capture_frame_rate;
        double*shutter,*brightness,*contrast,*sharpness,*gain;
         char*fmt,*ofmt;
        CameraDriverInterface*driver;
        void updateProperty();
        bool setCamera(const std::string &name, bool value, uint32_t size=sizeof(bool));
        bool setCamera(const std::string &name, int32_t value, uint32_t size=sizeof(bool));

        bool setDrvProp(const std::string &name, const int32_t value, uint32_t size=sizeof(int32_t));
        bool setDrvProp(const std::string &name, const double value, uint32_t size=sizeof(double));

        void startGrabbing();
        void haltThreads();
        void fatalErrorHandler(const chaos::CException & r);
        void stopGrabbing();
		/*!
		Define the Control Unit Dataset and Actions
		*/
		void unitDefineActionAndDataset()throw(chaos::CException);
		/*!(Optional)
		Define the Control Unit custom attribute
		*/
		void unitDefineCustomAttribute();
    /*!(Optional)
     Initialize the Control Unit and all driver, with received param from MetadataServer
     */
		void unitInit() throw(chaos::CException);
    /*!(Optional)
     Execute the work, this is called with a determinated delay
     */
    void unitStart() throw(chaos::CException);
    /*!
     Execute the work, this is called with a determinated delay, it must be as fast as possible
     */
    void unitRun() throw(chaos::CException);

    /*!(Optional)
     The Control Unit will be stopped
     */
    void unitStop() throw(chaos::CException);

    /*!(Optional)
     The Control Unit will be deinitialized and disposed
     */
    void unitDeinit() throw(chaos::CException);

		//! Pre input attribute change
		/*!(Optional)
		This handler is called befor the update of the
		cached input attribute with the requested valure
		*/
		bool unitInputAttributePreChangeHandler(chaos::common::data::CDWUniquePtr& data);

    chaos::common::data::CDWUniquePtr unitPerformCalibration(chaos::common::data::CDWUniquePtr data);

    virtual int filtering(cv::Mat&image);
    
    chaos::common::data::CDWUniquePtr getAction(chaos::common::data::CDWUniquePtr );
    chaos::common::data::CDWUniquePtr setAction(chaos::common::data::CDWUniquePtr );
    bool unitRestoreToSnapshot(chaos::cu::control_manager::AbstractSharedDomainCache * const snapshot_cache) throw(chaos::CException);

};
    }}}
#endif
