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


typedef enum AbstractSensorDriverOpcode{
	AbstractSensorDriverOpcode_READ_CHANNEL = cu_driver::OpcodeType::OP_USER,
	AbstractSensorDriverOpcode_WRITE_CHANNEL,
    AbstractSensorDriverOpcode_GET_DATASET,
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

/*
 driver definition
 */
class AbstractSensorDriver:ADD_CU_DRIVER_PLUGIN_SUPERCLASS {
	int32_t i32_out_1_value;
	void driverInit(const char *initParameter) throw(chaos::CException);
	void driverDeinit() throw(chaos::CException);
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
     \brief return the dataset of the sensor in *data, 
     deallocate after use
     \param sizen[in] max number of objects
     \param data[out] dataset array
     \return the number of sets, negative if error
     */
    virtual int getDataset(ddDataSet_t*data,int sizen)=0;
    
    
    
};

#endif /* defined(__ControlUnitTest__DummyDriver__) */
