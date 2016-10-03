#include "Arduino.h"
#include "SD2403RTC.h"

SD2403RTC::SD2403RTC() {

  Wire.begin();
}

String SD2403RTC::getTimeStamp() {

  unsigned char n=0;
 
  Wire.requestFrom(RTCAddress,7);
 
  while(Wire.available()){  
    date[n++]=Wire.read();
  }
  delayMicroseconds(1);
  Wire.endTransmission();

 unsigned char i;
 
  for(i=0;i<7;i++) {
    if(i!=2)
      date[i]=(((date[i]&0xf0)>>4)*10)+(date[i]&0x0f);
    else{
      date[2]=(date[2]&0x7f);
      date[2]=(((date[2]&0xf0)>>4)*10)+(date[2]&0x0f);
    }
  } // End for

  String timeStamp="";
  if(date[4]<10)
    timeStamp+=("0");
  timeStamp+=date[4]; // day
  timeStamp+=("/");
  if(date[5]<10)
    timeStamp+=("0");  
  timeStamp+=date[5]; // month
  timeStamp+=("/20");
  timeStamp+=date[6]; // year
  timeStamp+=(" ");
  if(date[2]<10)
    timeStamp+=("0");  
  timeStamp+=date[2]; // hour
  timeStamp+=(":");
  if(date[1]<10)
    timeStamp+=("0");  
  timeStamp+=date[1]; // minute
  
  return timeStamp;
}
