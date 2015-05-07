//////////////////////////////////////////////////////////////////////////////////
/*      End Node 2 Firmware

Informazioni:

Il datalogging avviene mediante file EndNode2.txt
N.b.
Il nome del file non deve superare i 12 caratteri, compresa l'estensione
Logica:

1. Se non esiste, viene creato il file;
2. Ogni entry viene aggiunta in coda al file


--------------------
Legend.

Debug messages:

M1 = SD Initialization failed  -->  setup() method
M2 = Writing to EndNode2.txt  -->  storeData() method
M3 = ERROR: Writing EndNode2.txt failed  -->  storeData() method
M4 = Starting new Session  -->  initSession() method
M5 = Error: Opening EndNode2.txt failed  -->  readData() method

--------------------
Error messages:

DHT11 sensors:

E1 = ERROR_Checksum
E2 = ERROR_Time out
E3 = ERROR_Unknown

DS18b20 sensors:

E1 = ERROR_CRC invalid
E2 = ERROR_Device not recognized
*/


///////////////////////////////////////////////////////////////////////////////
//  DHT11 sensors

#include <dht11.h>
const uint8_t dht11SensorsQty = 3; // Numero di sensori DHT11
const uint8_t dht11SensorsDIP [3] = {2,3,5}; // DIP cui sono connessi
dht11 dht11_1, dht11_2, dht11_3; // Istanzio i singoli oggetti
dht11 dht11Sensors[3] = {dht11_1, dht11_2, dht11_3}; // Dispongo gli oggetti in un array

///////////////////////////////////////////////////////////////////////////////
//  DS18B20 sensors

#include <OneWire.h>
const uint8_t ds18b20SensorsQty = 2; // Numero sonde ds18b20
const uint8_t ds18b20SensorsDIP [2] = {8,9}; // DIP cui sono connesse
OneWire ds18b20_1 (ds18b20SensorsDIP[0]); // Istanzio i singoli oggetti di tipo ds18b20
OneWire ds18b20_2 (ds18b20SensorsDIP[1]); 
OneWire ds18b20Sensors[] = {ds18b20_1, ds18b20_2}; // Dispongo gli oggetti in un array

///////////////////////////////////////////////////////////////////////////////
//  MicroSD card module

#include <SD.h>
File EndNode2LogFile;
const uint8_t chipSelect=SS; // SD chip select pin

///////////////////////////////////////////////////////////////////////////////
//  SD2403RTC Module

#include <Wire.h>
#include <SD2403RTC.h>
SD2403RTC rtcModule;

///////////////////////////////////////////////////////////////////////////////
//  Variables

const int sensorsSleepTime = 3000; // Attesa nei cicli for per ogni sensore
const int sampleInterval = 20000; // Intervallo di campionamento per ogni ciclo



///////////////////////////////////////////////////////////////////////////////
/*  Setup() method

Initialize session

*/

void setup() {

  delay(3000); // Attendo il warm up del modulo ZigBee  
  Serial.begin(9600);
  if(!SD.begin(chipSelect)) {
    Serial.println(F("*** E2 - M1")); // SD Initialization failed
    Serial.flush();
  }
//  initSession();
}


///////////////////////////////////////////////////////////////////////////////
/*  loop() method

The End Node Device sends sensor data when queried

*/
void loop() {

  delay(sampleInterval);
  printDht11Sensors();  
  
  delay(sensorsSleepTime);
  printDs18b20Sensors();   
}


///////////////////////////////////////////
/*  printDht11Sensors() method

Algoritmo Round Robin sui sensori DHT11.
1. Inizializza la stringa di output
2. Interroga l'i-mo sensore
3. Aggiorna la stringa di output
4. Storicizza ed invia sulla seriale la stringa di output

*/

void printDht11Sensors(){
  
  String dht11Data="E 2 ";
  for (uint8_t i=0; i<dht11SensorsQty; i++) {
    dht11Data+="D ";
    dht11Data+=i+1;
    dht11Data+=" ";
    
    switch (dht11Sensors[i].read(dht11SensorsDIP[i])) {

      case DHTLIB_OK:
        dht11Data+=dht11Sensors[i].temperature;
        dht11Data+=" ";
        dht11Data+=dht11Sensors[i].humidity;
        dht11Data+=" ";
        break;
      case DHTLIB_ERROR_CHECKSUM: 
        dht11Data+="E1 ";
        break;
      case DHTLIB_ERROR_TIMEOUT:
        dht11Data+="E2 ";
        break;
      default:
        dht11Data+="E3 ";
        break;
    }  // End switch
  
    delay(sensorsSleepTime);
  } // End for

  dht11Data+=rtcModule.getTimeStamp();
  storeData(dht11Data);
  Serial.println(dht11Data);
  Serial.flush();

} // End printDht11Sensors() method


///////////////////////////////////////////
/*  printDs18b20Sensors() method

Algoritmo Round Robin sulle sonde DS18B20.
1. Inizializza la stringa di output
2. Interroga l'i-mo sensore
3. Aggiorna la stringa di output
4. Storicizza ed invia sulla seriale la stringa di output

*/

void printDs18b20Sensors(){
  
  String ds18b20Data="E 2 ";
  for (int i=0; i<ds18b20SensorsQty;i++) {
    ds18b20Data+="S ";
    ds18b20Data+=i+1;
    ds18b20Data+=" ";
    
    byte data[12];
    byte addr[8];

    if ( !ds18b20Sensors[i].search(addr))
      //no more sensors on chain, reset search
      ds18b20Sensors[i].reset_search();

    if ( OneWire::crc8( addr, 7) != addr[7])
      ds18b20Data+="E1 ";       

    if (addr[0] != 0x10 && addr[0] != 0x28)
      ds18b20Data+="E2 ";

    ds18b20Sensors[i].reset();
    ds18b20Sensors[i].select(addr);
    ds18b20Sensors[i].write(0x44,1); // start conversion, with parasite power on at the end

    byte present = ds18b20Sensors[i].reset();
    ds18b20Sensors[i].select(addr);    
    ds18b20Sensors[i].write(0xBE); // Read Scratchpad

    for (int k = 0; k < 9; k++) // we need 9 bytes
      data[k] = ds18b20Sensors[i].read();

    ds18b20Sensors[i].reset_search();

    byte MSB = data[1];
    byte LSB = data[0];

    float tempRead = ((MSB << 8) | LSB); //using two's compliment
    float TemperatureSum = tempRead / 16;
    
    char tempChar[5];
    dtostrf(TemperatureSum,4,3,tempChar);
    
    for(int j=0; j<sizeof(tempChar); j++)
      ds18b20Data+=tempChar[j];
    
    ds18b20Data+=" ";
    
    delay(sensorsSleepTime);
  }// End for  

  ds18b20Data+=rtcModule.getTimeStamp();
  storeData(ds18b20Data);
  Serial.println(ds18b20Data);
  Serial.flush();

}// End printDs18b20Sensors() method


///////////////////////////////////////////
/*  storeData() method

Storicizza la stringa su microSD card.
1. Apre il file di log
2. Appende in coda la nuova stringa
3. Chiude il file

*/

void storeData (String str){
  EndNode2LogFile = SD.open("EndNode2.txt", FILE_WRITE);
 
  if (EndNode2LogFile){ 
    EndNode2LogFile.println(str); // Append new record to file
    EndNode2LogFile.close();
  }
  else {
    Serial.println(F("*** E2 - M3"));// ERROR: Writing EndNode2.txt failed
    Serial.flush(); 
  }  
}


///////////////////////////////////////////
/*  initiSession() method

Marca sul file di log l'inizio di una nuova sessione di acquisizione

*/
/*
void initSession() {

  String str ="*** E2 - Starting new session: ";
  str+=rtcModule.getTimeStamp();
  Serial.println(str);
  Serial.flush();
  storeData(str);
}
*/

///////////////////////////////////////////
/*  freeRam() method

Ritorna il numero di byte della RAM ancora diponibili

*/
/*
int freeRam () {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}
*/
