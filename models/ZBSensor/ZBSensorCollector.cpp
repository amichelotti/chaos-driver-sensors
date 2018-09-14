/*
 *	ZBSensorCollector.h
 *	!CHAOS
 *	Created by Andrea Michelotti.
 *
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
#include <driver/sensors/models/ZBSensor/ZBSensorCollector.h>
#include <boost/bind.hpp>
#include <stdlib.h>
#include <string>
using namespace driver::sensor::model;

#define MAX_BUF 8192

#define ZBSensorCollectorLAPP_		LAPP_ << "[ZBSensorCollector] "
#define ZBSensorCollectorLDBG_		LDBG_ << "[ZBSensorCollector] "
#define ZBSensorCollectorLERR_		LERR_ << "[ZBSensorCollector] "
/*
  std::map<int ,boost::lockfree::queue<std::string>* >
     E 1 D 1 23 56 D 2 23 59 D 3 24 50 02/04/2015 11:02
    
Come interpretarla: (Dedicata ai sensori per misure combinate di Temp ed umidità dell'aria)
                    
E 1 -> Stringa spedita dall'EndNode 1
D 1 23 56 -> Il sensore di temperatura ed umidita numero 1 ha rilevato Temperatura=23 °C ed Umidità=56 %
D 2 23 59 -> Il sensore di temperatura ed umidita numero 2 ha rilevato Temperatura=23 °C ed Umidità=59 %
D 3 23 50 -> Il sensore di temperatura ed umidita numero 3 ha rilevato Temperatura=24 °C ed Umidità=50 %
02/04/2015 11:02 -> Timestamp della misurazione con risoluzione al minuto (E' possibile aggiungere anche i secondi ma non credo ce ne sia bisogno data la natura delle grandezze fisiche monitorate)

                    
    E 3 S 1 23.58 S 2 23.05 S 3 23.69 01/04/2015 12:00
                    
Come interpretarla: (Dedicata ai sensori per misure di Temperatura superficiale delle pareti)
                    
E 3 -> Stringa spedita dall'EndNode 3
S 1 23.58 -> La sonda di temperatura superficiale numero 1 ha rilevato Temperatura=23.58 °C
S 2 23.05 -> La sonda di temperatura superficiale numero 2 ha rilevato Temperatura=23.05 °C
S 3 23.69 -> La sonda di temperatura superficiale numero 3 ha rilevato Temperatura=23.69 °C
01/04/2015 12:00 -> Timestamp della misurazione con risoluzione al minuto (E' possibile aggiungere anche i secondi ma non credo ce ne sia bisogno data la natura delle grandezze fisiche monitorate)

                    
    E 3 C 1 25567 01/04/2015 12:00
    E 1 C 1 25567 01/04/2015 12:00
Come interpretarla: (Dedicata ai sensori per misure concentrazione CO2)
                    
E 3 -> Stringa spedita dall'EndNode 3
C 1 25567 -> Il sensore per la concentrazione di CO2 numero 1 ha rilevato una concentrazione di 25567 PPM                    
01/04/2015 12:00 -> Timestamp della misurazione con risoluzione al minuto (E' possibile aggiungere anche i secondi ma non credo ce ne sia bisogno data la natura delle grandezze fisiche monitorate)  
 */

//GET_PLUGIN_CLASS_DEFINITION
//we need to define the driver with alias version and a class that implement it

void ZBSensorCollector::init(std::string param){
    exit =0;
   if(serial==NULL){
    
    serial = new common::serial::PosixSerialComm(param,9600,0,8,1,0,MAX_BUF,MAX_BUF);
    assert(serial);
    if(serial->init()!=0){
      throw chaos::CException(-1, "Cannot initialize serial device:"+param, __FUNCTION__);
    }
    ZBSensorCollectorLDBG_ << "Initialized serial :"<<param<<" pnt:"<<serial;
   } else {
    ZBSensorCollectorLDBG_ << "Already Initialized serial :"<<param<<" pnt:"<<serial;

   }
    
    char test[2];
    test[0]='\n';
    test[1]='\r';
    serial->write(&test,1);
    if(collector_thread==NULL){
        collector_thread = new boost::thread(boost::bind(&ZBSensorCollector::updateStatus,this));
        assert(collector_thread);
    }
}

void ZBSensorCollector::deinit(){
    if(exit==0){
        exit =1;
        if(serial){
            delete serial;
            serial = NULL;
        }
        if(collector_thread){
            delete collector_thread;
            collector_thread =NULL;
        }
         std::map<int ,std::queue<zbnodedata_t> * >::iterator i;
         for(i=zb_queue.begin();i!=zb_queue.end();i++){
             delete (i->second);
         }
    }
}

ZBSensorCollector::ZBSensorCollector(){
   exit =1;
   serial = NULL;
   collector_thread = NULL;
}

ZBSensorCollector::ZBSensorCollector(std::string param){
  
    init(param);
}
//default descrutcor
ZBSensorCollector::~ZBSensorCollector() {
    deinit();
}
void ZBSensorCollector::updateStatus(){
    char buffer[MAX_BUF];
    int cnt=0;
    ZBSensorCollectorLDBG_ << "update status thread started.";
    while(exit==0){
        if(serial->read(&buffer[cnt],1)>0){
          if((cnt==(sizeof(buffer)-1)) || buffer[cnt]=='\n' || buffer[cnt]=='\r') {
            int node=0;
            int channel=0; 
            int parsecnt=0;
            int uid=0;
            int sensors=0;
            buffer[cnt]=0;
            int type;
            int cntt;
            char*pnt;
            zbnodedata_t data[MAX_ZB_SENSOR_CHANNELS];
            
            pnt = buffer;
            for(pnt=buffer;(pnt<buffer+cnt)&&(*pnt!='E');pnt++);
            ZBSensorCollectorLDBG_ << "received buffer["<<cnt<<","<<strlen(buffer)<<"]:\""<<pnt<<"\"";
    
            pnt=strtok(pnt," ");
            
            while((pnt!=NULL)/*&& (*pnt!=0)*/){
                if(*pnt == 'E'){
                    parsecnt=0;
                } else if((*pnt == 'C') || (*pnt == 'D') || (*pnt == 'S')){
                   
                    parsecnt=2;
                }
                switch(parsecnt){
                    case 0:
                        break;
                    case 1:
                        //get node
                        node = atoi(pnt);
                       ZBSensorCollectorLDBG_<<"node:"<<node;

                        break;
                    case 2:
                        // type;
                        type = *pnt;
                        sensors++;
                        break;
                    case 3:
                    // get the channel
                        channel = atoi(pnt);
                        uid = (type << 16) | (node<<8) | (channel&0xFF); 
                        ZBSensorCollectorLDBG_<<"channel:"<<channel<<"node:"<<node<<" type:"<<type<<" uid:"<<std::hex<<uid<<std::dec;
                        if(uid==0) {
                           ZBSensorCollectorLERR_ << "bad uid something is going wrong:"<<buffer;
                           *pnt=0;
                           continue;
                        } else {
                            if(zb_queue.find(uid)==zb_queue.end()){
                                ZBSensorCollectorLDBG_<<"new queue created for sensor:"<<std::hex<<uid<<std::dec;
                                zb_queue[uid]=new std::queue<zbnodedata_t>();
                            }
                        }
                        data[sensors-1].uid=uid;

                        break;
                    default:
                    // get the values
                        if(parsecnt<MAX_ZB_SENSOR_CHANNELS+4){
                            if(strchr(pnt,'/')){
                                strncpy(data[sensors-1].data,pnt,16);
                            } else if(strchr(pnt,':')){
                                strncpy(data[sensors-1].hour,pnt,16);
                                
                            } else {
                                if(parsecnt-4 >= 0 ){
                                    data[sensors-1].sensor[parsecnt-4]=atof(pnt);
                                    data[sensors-1].nsensors++;
                                    ZBSensorCollectorLDBG_ << "acquiring sensor "<<sensors<<" channel:\""<<channel<<"\" in queue of sensorid:"<<std::hex<<uid<<std::dec<< " value:"<< data[sensors-1].sensor[parsecnt-4];
                                }
                            }
                        }
                        break;
              
                }
            
            parsecnt++;
            pnt=strtok(NULL," ");
            }
            if(*(data[sensors-1].data)!=0){
                int cnt,cntt;
                for(cnt=0;cnt<sensors;cnt++){
                    strncpy(data[cnt].data,data[sensors-1].data,16);
                    strncpy(data[cnt].hour,data[sensors-1].hour,16);
                    zb_queue[data[cnt].uid]->push(data[cnt]);
                    
                    ZBSensorCollectorLDBG_ << "pushing "<<data[cnt].nsensors<<" sensors, acquired at:\""<<data[cnt].data<<"\" in queue of sensorid:"<<data[cnt].uid;
                    for(cntt=0;cntt<data[cnt].nsensors;cntt++){
                        ZBSensorCollectorLDBG_ << "\t-"<<data[cnt].sensor[cntt];
                    }
                }
            }
             
               cnt =0;
         }
          cnt++;
              
        }
    }
        ZBSensorCollectorLDBG_ << "update status thread exit";

}

zbnodedata_t ZBSensorCollector::getNode(int id){
    std::map<int ,std::queue<zbnodedata_t> * >::iterator i=zb_queue.find(id);
    zbnodedata_t ret;
  //   ZBSensorCollectorLDBG_<<"get Node id:"<<std::hex<<id<<std::dec;
    if(i != zb_queue.end()){
        if(!i->second->empty()){
            ret=i->second->front();
            i->second->pop();
            
        }
    }
     return ret;
}

