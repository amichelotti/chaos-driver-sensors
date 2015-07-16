//
//  SensorDriverInterface.h
//  modbus
//
//  Created by andrea michelotti on 03/09/14.
//  Copyright (c) 2014 andrea michelotti. All rights reserved.
//

#ifndef __modbus__SensorDriverInterface__
#define __modbus__SensorDriverInterface__

#include <iostream>
#include <driver/sensors/core/AbstractSensorDriver.h>
#include <chaos/cu_toolkit/driver_manager/driver/DriverTypes.h>
#include <chaos/cu_toolkit/driver_manager/driver/DriverAccessor.h>

class SensorDriverInterface:public ::driver::sensors::AbstractSensorDriver{
                
            protected:
                cu_driver::DrvMsg message;
                
            public:
                
                SensorDriverInterface(cu_driver::DriverAccessor*_accessor):accessor(_accessor){assert (_accessor);};
                
                cu_driver::DriverAccessor* accessor;
                
                /**
                 \brief Read a channel from the physical sensor
                 \param buffer[out] destination buffer
                 \param addr[in] channel address or identification
                 \param bcout[in] buffer count
                 \return the number of succesful read items, negative error
                 
                 */

                
                int readChannel(void *buffer,int addr,int bcount);
                /**
                 \brief Write a channel from the physical sensor
                 \param buffer[out] destination buffer
                 \param addr[in] channel address or identification
                 \param bcout[in] buffer count
                 \return the number of succesful written items, negative error
                 */
                int writeChannel(void *buffer,int addr,int bcount);
                
                /**
                 \brief init the sensor
                 \param buffer[in] initialisation opaque parameter
                 \return 0 if success, error otherwise
                 
                 */
                int sensorInit(void *buffer,int sizeb);
                
                /**
                 \brief deinit the sensor
                 \param buffer[in] initialisation opaque parameter
                 \return 0 if success, error otherwise
                 */

                int sensorDeinit();

                /**
                 \brief return the dataset of the sensor in *data,
                 deallocate after use
                 \param sizen[in] max number of objects
                 \param data[out] dataset array
                 \return the number of sets, negative if error
                 */
                int getDataset(::driver::sensors::ddDataSet_t*data,int sizen);

		/**
		   \brief return the size in byte of the dataset
		   \return the size of the dataset if success, zero otherwise
		*/
		int getDatasetSize();


                
            };
            
    


#endif /* defined(__modbus__SensorDriverInterface__) */
