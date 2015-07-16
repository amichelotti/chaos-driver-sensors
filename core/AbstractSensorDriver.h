/*
 *	AbstracSensorDriver.h
 *	!CHAOS
 *	Created by Andrea Michelotti
 *  Abstraction of a simple sensor driver
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
#ifndef __ASTRACTSENSORDRIVER_H__
#define __ASTRACTSENSORDRIVER_H__
#include <chaos/cu_toolkit/driver_manager/driver/AbstractDriverPlugin.h>

namespace cu_driver = chaos::cu::driver_manager::driver;

namespace driver {
    
    namespace sensors {
        
    
typedef enum AbstractSensorDriverOpcode{
	AbstractSensorDriverOpcode_READ_CHANNEL = cu_driver::OpcodeType::OP_USER,
	AbstractSensorDriverOpcode_WRITE_CHANNEL,
	AbstractSensorDriverOpcode_GET_DATASET,
	AbstractSensorDriverOpcode_GET_DATASETSIZE,
	AbstractSensorDriverOpcode_INIT,
	AbstractSensorDriverOpcode_DEINIT

} AbstractSensorDriverOpcode;


typedef struct ddDataSet {
    const char* name;
    const char* desc;
    chaos::DataType::DataSetAttributeIOAttribute dir;
    chaos::DataType::DataType type;
    int maxsize;
} ddDataSet_t;

#define DEF_SENSOR_DATASET \
  static ::driver::sensors::ddDataSet_t dataSet[]={


#define DEF_SENSOR_CHANNEL(_name,_desc,_dir,_type,_size)	\
  {_name,_desc,_dir, _type,_size},
#define ENDDEF_SENSOR_DATASET };
#define INIT_SENSOR_DATASET setDataSet(dataSet,sizeof(dataSet));
/*
 driver definition
 */

class AbstractSensorDriver:ADD_CU_DRIVER_PLUGIN_SUPERCLASS {

	void driverInit(const char *initParameter) throw(chaos::CException);
	void driverDeinit() throw(chaos::CException);
protected:
    ddDataSet_t *dataset;
    int datasetSize;
public:
	AbstractSensorDriver();

	~AbstractSensorDriver();
    //! Execute a command
	cu_driver::MsgManagmentResultType::MsgManagmentResult execOpcode(cu_driver::DrvMsgPtr cmd);
        /**
         \brief Read a channel from the physical sensor
         \param buffer[out] destination buffer
         \param addr[in] channel address or identification
         \param bcout[in] buffer count
         \return the number of succesful read items, negative error

         */
    virtual int readChannel(void *buffer,int addr,int bcount)=0;
    /**
     \brief Write a channel from the physical sensor
     \param buffer[out] destination buffer
     \param addr[in] channel address or identification
     \param bcout[in] buffer count
     \return the number of succesful written items, negative error
     */
    virtual int writeChannel(void *buffer,int addr,int bcount)=0;
    
    /**
     \brief init the sensor
     \param buffer[in] initialisation opaque parameter
     \return 0 if success, error otherwise
     
     */
    virtual int sensorInit(void *buffer,int sizeb)=0;
    
    /**
     \brief deinit the sensor
     \param buffer[in] initialisation opaque parameter
     \return 0 if success, error otherwise
     */
    virtual int sensorDeinit()=0;
    /**
       \brief return the size in byte of the dataset
       \return the size of the dataset if success, zero otherwise
     */
     int getDatasetSize();

    /**
       \brief return the dataset copying max size bytes
       \param data[out] array of data
       \param sizeb[in] max byte to copy
       \return the size of the dataset if success, zero otherwise
     */

    int getDataset(ddDataSet_t*data,int sizeb);
    
    void setDataSet(ddDataSet_t*data,int sizeb);
};
    }
}
#endif /* defined(__ControlUnitTest__DummyDriver__) */
