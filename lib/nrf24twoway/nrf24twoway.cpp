
#include "nrf24twoway.h"


nrf24twoway::nrf24twoway() {

}


void nrf24twoway::begin(uint8_t transmiter) {
  radio.begin();
  radio.setDataRate( RF24_250KBPS );
  radio.setPALevel(RF24_PA_MIN);
  switch (transmiter) {
    case 1:
      radio.openReadingPipe(1, radioAddress[1]);
      radio.openWritingPipe(radioAddress[0]);
      break;
    case 2:
      radio.openReadingPipe(1, radioAddress[0]);
      radio.openWritingPipe(radioAddress[1]);
      break;
    default:
      Serial.println("Transmiter number invalid");
      break;
  }


  radio.startListening();
}

int nrf24twoway::getData(String *buf)
{
  if ( radio.available() )
  {
    radio.read( &dataReceived, sizeof(dataReceived) );
    *buf = dataReceived;
    memset(dataReceived, 0, sizeof(dataReceived));
    return 1;
  }
  return 0;

}

void nrf24twoway::sendData(String *buf2) {
  int i = 0;
  radio.stopListening();
  char* test2 = new char[buf2->length() + 1];
  strcpy(test2, buf2->c_str());
  radio.write(test2, sizeof(test2));
  delete[] test2;
  radio.startListening();

}
