#ifndef nrf24twoway_h
#define nrf24twoway_h

#include <Arduino.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#define charsize 30                               // Max chars for transmitting
#define cepin 27                                  // CE Pin NRF24 Modul
#define csnpin 14                                 // CSN Pin NRF24 Modul



class nrf24twoway {
  public:
    nrf24twoway();
    void begin(uint8_t transmiter);
    void sendData(String *buf2);
    int getData(String *buf);
    char dataSend[charsize] ;
    char dataReceived[charsize];

    const byte radioAddress[2][6] = {"00001", "00002"};      //2 transmiter adress


    RF24 radio = RF24(cepin, csnpin);

  private:


};











#endif
