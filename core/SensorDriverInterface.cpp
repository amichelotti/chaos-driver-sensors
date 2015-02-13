//
//  ChaosModbusInterface.cpp
//  modbus
//
//  Created by andrea michelotti on 03/09/14.
//  Copyright (c) 2014 andrea michelotti. All rights reserved.
//

#include "SensorDriverInterface.h"
#include <chaos/common/exception/CException.h>

#define DATASET_SIZE_BUFFER 1024*8





int SensorDriverInterface::readChannel(void *buffer,int addr,int bcount){
    
    int ret,ret2;
    message.parm[0] = addr;
    message.parm[1] = bcount;
    message.opcode =AbstractSensorDriverOpcode_READ_CHANNEL;
    message.resultDataLength=bcount;
    message.resultData = (void*)buffer;
    ret2=accessor->send(&message);
    ret=message.ret;
    LDBG_<<"readChannel addr:"<<addr<<", count:"<<bcount<<",func ret:"<<ret<<",accessor ret "<<ret2;
    return ret;
    
}

int SensorDriverInterface::writeChannel(void *buffer,int addr,int bcount){
    
    int ret,ret2;
    message.parm[0] = addr;
    message.parm[1] = bcount;
    message.opcode =AbstractSensorDriverOpcode_WRITE_CHANNEL;
    message.inputDataLength=bcount;
    message.inputData = (void*)buffer;
    ret2=accessor->send(&message);
    ret=message.ret;
    LDBG_<<"writeChannel addr:"<<addr<<", count:"<<bcount<<",func ret:"<<ret<<",accessor ret "<<ret2;
    return ret;

}

int SensorDriverInterface::sensorInit(void *buffer,int sizeb){
    int ret,ret2;
    message.parm[0] = sizeb;
    message.opcode =AbstractSensorDriverOpcode_INIT;
    message.inputDataLength=sizeb;
    ret2=accessor->send(&message);
    ret=message.ret;
    LDBG_<<"Init,func ret:"<<ret<<",accessor ret "<<ret2;
    return ret;


}

int SensorDriverInterface::sensorDeinit(){
    int ret,ret2;
    
    message.opcode =AbstractSensorDriverOpcode_DEINIT;
    message.inputDataLength=0;
    ret2=accessor->send(&message);
    ret=message.ret;
    LDBG_<<"DeInit,func ret:"<<ret<<",accessor ret "<<ret2;
    return ret;
}

int SensorDriverInterface::getDataset(ddDataSet_t*data,int sizen){
    int ret2;
    int size=0;
    message.resultData = (void*)data;
    message.resultDataLength = sizeof(ddDataSet_t)*sizen;
    message.opcode =AbstractSensorDriverOpcode_GET_DATASET;
    message.inputDataLength=0;
    ret2=accessor->send(&message);
    size=message.ret;

    LDBG_<<"getDataset,func ret:"<<size<<",accessor ret "<<ret2;
    return size;
   
}


int SensorDriverInterface::getDatasetSize(){
  int size,ret2;
    message.resultData = 0;
    message.resultDataLength = 0;
    message.opcode =AbstractSensorDriverOpcode_GET_DATASETSIZE;
    message.inputDataLength=0;
    ret2=accessor->send(&message);
    size=message.ret;

    LDBG_<<"getDatasetSize,func ret:"<<size<<",accessor ret "<<ret2;
    return size;
    

}
