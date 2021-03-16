#include <Arduino.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Adafruit_PWMServoDriver.h>
#include "hexapod.h"
#include "nrf24twoway.h"                            // make sure to set the CE and CSN pins in nrf24twoway.h file
#include "StringSplitter.h"



//#define ENABLE_DEBUG1

int zTest = 30;
int speed = 60;
String radioData;
String yes = "yes";
String reset = "reset";

String badRange = "Bad range";
String noCommand = "No command detected";

nrf24twoway funk;

class hexapod paul;

enum servos { front_left_coax, front_left_femur, front_left_tibia,
              middle_left_coax, middle_left_femur, middle_left_tibia,
              rear_left_coax, rear_left_femur, rear_left_tibia,
              front_right_coax, front_right_femur, front_right_tibia,
              middle_right_coax, middle_right_femur, middle_right_tibia,
              rear_right_coax, rear_right_femur, rear_right_tibia
            };
enum legs { front_left,
            middle_left,
            rear_left,
            front_right,
            middle_right,
            rear_right
          };


void setup() {

  Serial.begin(115200);
  delay(1000);
  Serial.println("Start");
  funk.begin(2);                 // 1 for sender 2 for receiver


  paul.begin();

  Wire.setClock(400000);


  // Servo, Pin, AngleOffset, Flipped
  paul.attachServo(front_left_coax,     8, -36 , 0);
  paul.attachServo(front_left_femur,    9, 0, 0);
  paul.attachServo(front_left_tibia,    10, 0, 1);
  paul.attachServo(middle_left_coax,    23, 2 , 0);
  paul.attachServo(middle_left_femur,   22, 0, 0);
  paul.attachServo(middle_left_tibia,   21, 0, 1);
  paul.attachServo(rear_left_coax,      19, 35 , 0);
  paul.attachServo(rear_left_femur,     13, 0, 0);
  paul.attachServo(rear_left_tibia,     17, 0, 1);
  paul.attachServo(front_right_coax,    7, -40, 1);
  paul.attachServo(front_right_femur,   6, 0, 1);
  paul.attachServo(front_right_tibia,   5, 0, 0);
  paul.attachServo(middle_right_coax,   24, 0, 1);
  paul.attachServo(middle_right_femur,  25, 0, 1);
  paul.attachServo(middle_right_tibia,  26, 0, 0);
  paul.attachServo(rear_right_coax,     28, 35, 1);
  paul.attachServo(rear_right_femur,    1, 0, 1);
  paul.attachServo(rear_right_tibia,    30, -4, 0);

  // X , Y , Z , Xoffset , Yoffset , Zoffset

  /*
    paul.init_koor(front_left,    80, -60, 80, 57.362, -84.853, 0);
    paul.init_koor(middle_left,   100, 0, 80, 71.90, 0, 0);
    paul.init_koor(rear_left,     80, 60, 80, 57.362, 84.853, 0);
    paul.init_koor(front_right,   -80, -60, 80, -57.362, -84.853, 0);
    paul.init_koor(middle_right,  -100, 0, 80, -71.90, 0, 0);
    paul.init_koor(rear_right,    -80, 60, 80, -57.362, 84.853, 0);
  */

  paul.init_koor(front_left,    80, -65, 100, 57.362, -84.853, 0);
  paul.init_koor(middle_left,   100, 0, 100, 71.90, 0, 0);
  paul.init_koor(rear_left,     80, 65, 100, 57.362, 84.853, 0);
  paul.init_koor(front_right,   -80, -65, 100, -57.362, -84.853, 0);
  paul.init_koor(middle_right,  -100, 0, 100, -71.90, 0, 0);
  paul.init_koor(rear_right,    -80, 65, 100, -57.362, 84.853, 0);

  //how many steps
  paul.set_steps(70);

  //coax, femur and tibia length
  paul.init_leg(50, 74.3, 120.5);
  //max and min Range of Angle
  paul.init_leg_range(5, 175, 5, 175, 5, 175);


  //Write init_koor
  for (int legNumber = 0; legNumber < 6; legNumber++) {
    paul.calculateW((int)legNumber);
    paul.writeServo(legNumber);

  }

}



void loop() {

  parseCom();


}

void parseCom() {
  int value[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  if (funk.getData(&radioData)) {
    StringSplitter *splitter = new StringSplitter(radioData, ',', 8);  // new StringSplitter(string_to_split, delimiter, limit)
    int itemCount = splitter->getItemCount();

#ifdef ENABLE_DEBUG1
    Serial.println("Test String: " + radioData);
    Serial.println("Item count: " + String(itemCount));
#endif

    for (int i = 0; i < itemCount; i++) {
      String item = splitter->getItemAtIndex(i);
      value[i] = item.toInt();
    }
    delete splitter;


#ifdef ENABLE_DEBUG1
    for (int i = 0; i < sizeof(value) / sizeof(value[0]); i++) {
      Serial.println(value[i]);
    }
#endif



    switch (value[0]) {
      case 1:
        paul.init_koor(front_left,    80, -65, 100, 57.362, -84.853, 0);
        paul.init_koor(middle_left,   100, 0, 100, 71.90, 0, 0);
        paul.init_koor(rear_left,     80, 65, 100, 57.362, 84.853, 0);
        paul.init_koor(front_right,   -80, -65, 100, -57.362, -84.853, 0);
        paul.init_koor(middle_right,  -100, 0, 100, -71.90, 0, 0);
        paul.init_koor(rear_right,    -80, 65, 100, -57.362, 84.853, 0);
        for (int legNumber = 0; legNumber < 6; legNumber++) {
          paul.calculateW((int)legNumber);
          paul.writeServo(legNumber);

        }
        funk.sendData(&reset);
        break;

      case 2:

        paul.movetoall(value[2], value[3], value[4], value[5]);
        funk.sendData(&yes);
        break;

      case 3:

        paul.introwalk(value[2], value[3], value[4], value[5]);
        paul.walk(value[1], value[2], value[3], value[4], value[5]);
        paul.outrowalk(value[2], value[3], value[4], value[5]);

        funk.sendData(&yes);
        break;

      case 4:

        for (int i = 0; i <= 1; i++) {
          paul.leg[middle_left].femurw = 160;
          paul.leg[middle_left].tibiaw = 160;

          paul.leg[middle_right].coaxw = -90;
          paul.leg[middle_right].femurw = 160;
          paul.leg[middle_right].tibiaw = 160;
          paul.writeServo(middle_left);
          paul.writeServo(middle_right);
          delay(300);


          paul.leg[middle_left].tibiaw = 140;

          paul.leg[middle_right].coaxw = -90;
          paul.leg[middle_right].femurw = 160;
          paul.leg[middle_right].tibiaw = 140;
          paul.writeServo(middle_left);
          paul.writeServo(middle_right);
          delay(300);

          paul.leg[middle_left].tibiaw = 160;
          paul.leg[middle_right].coaxw = -90;
          paul.leg[middle_right].femurw = 160;
          paul.leg[middle_right].tibiaw = 160;
          paul.writeServo(middle_left);
          paul.writeServo(middle_right);
          delay(300);

          paul.leg[middle_left].tibiaw = 140;
          paul.leg[middle_right].coaxw = -90;
          paul.leg[middle_right].femurw = 160;
          paul.leg[middle_right].tibiaw = 140;
          paul.writeServo(middle_left);
          paul.writeServo(middle_right);
          delay(300);

          paul.leg[middle_left].tibiaw = 160;
          paul.leg[middle_right].coaxw = -90;
          paul.leg[middle_right].femurw = 160;
          paul.leg[middle_right].tibiaw = 160;
          paul.writeServo(middle_left);
          paul.writeServo(middle_right);
          delay(300);

          paul.leg[middle_left].tibiaw = 140;
          paul.leg[middle_right].coaxw = -90;
          paul.leg[middle_right].femurw = 160;
          paul.leg[middle_right].tibiaw = 140;
          paul.writeServo(middle_left);
          paul.writeServo(middle_right);
          delay(300);

          paul.leg[middle_left].tibiaw = 160;
          paul.leg[middle_right].coaxw = -90;
          paul.leg[middle_right].femurw = 160;
          paul.leg[middle_right].tibiaw = 160;
          paul.writeServo(middle_left);
          paul.writeServo(middle_right);
          delay(300);

          paul.leg[middle_left].tibiaw = 140;
          paul.leg[middle_right].coaxw = -90;
          paul.leg[middle_right].femurw = 160;
          paul.leg[middle_right].tibiaw = 140;
          paul.writeServo(middle_left);
          paul.writeServo(middle_right);
          delay(300);

          paul.leg[middle_left].tibiaw = 160;
          paul.leg[middle_right].coaxw = -90;
          paul.leg[middle_right].femurw = 160;
          paul.leg[middle_right].tibiaw = 160;
          paul.writeServo(middle_left);
          paul.writeServo(middle_right);
          delay(300);



          paul.init_koor(front_left,    80, -65, 100, 57.362, -84.853, 0);
          paul.init_koor(middle_left,   100, 0, 100, 71.90, 0, 0);
          paul.init_koor(rear_left,     80, 65, 100, 57.362, 84.853, 0);
          paul.init_koor(front_right,   -80, -65, 100, -57.362, -84.853, 0);
          paul.init_koor(middle_right,  -100, 0, 100, -71.90, 0, 0);
          paul.init_koor(rear_right,    -80, 65, 100, -57.362, 84.853, 0);
          for (int legNumber = 0; legNumber < 6; legNumber++) {
            paul.calculateW((int)legNumber);
            paul.writeServo(legNumber);

          }
          delay(500);
        }
        for (int i = 0; i <= 1; i++) {
          //
          //      Rechte Seite 4x Rauf runter
          //
          paul.settoleg(front_left,   0,    0,  -20, 0);
          paul.settoleg(middle_left,  0,    0,    0, 0);
          paul.settoleg(rear_left,    0,    0,  -20, 0);
          paul.settoleg(front_right,  0,    0,    0, 0);
          paul.settoleg(middle_right, 0,    0,  -20, 0);
          paul.settoleg(rear_right,   0,    0,    0, 0);
          paul.movelegs(speed);

          paul.settoleg(front_left,   0,    0,  20, 0);
          paul.settoleg(middle_left,  0,    0,    0, 0);
          paul.settoleg(rear_left,    0,    0,  20, 0);
          paul.settoleg(front_right,  0,    0,    0, 0);
          paul.settoleg(middle_right, 0,    0,  20, 0);
          paul.settoleg(rear_right,   0,    0,    0, 0);
          paul.movelegs(speed);

          paul.settoleg(front_left,   0,    0,  -20, 0);
          paul.settoleg(middle_left,  0,    0,    0, 0);
          paul.settoleg(rear_left,    0,    0,  -20, 0);
          paul.settoleg(front_right,  0,    0,    0, 0);
          paul.settoleg(middle_right, 0,    0,  -20, 0);
          paul.settoleg(rear_right,   0,    0,    0, 0);
          paul.movelegs(speed);

          paul.settoleg(front_left,   0,    0,  20, 0);
          paul.settoleg(middle_left,  0,    0,    0, 0);
          paul.settoleg(rear_left,    0,    0,  20, 0);
          paul.settoleg(front_right,  0,    0,    0, 0);
          paul.settoleg(middle_right, 0,    0,  20, 0);
          paul.settoleg(rear_right,   0,    0,    0, 0);
          paul.movelegs(speed);

          paul.settoleg(front_left,   0,    0,  -20, 0);
          paul.settoleg(middle_left,  0,    0,    0, 0);
          paul.settoleg(rear_left,    0,    0,  -20, 0);
          paul.settoleg(front_right,  0,    0,    0, 0);
          paul.settoleg(middle_right, 0,    0,  -20, 0);
          paul.settoleg(rear_right,   0,    0,    0, 0);
          paul.movelegs(speed);

          paul.settoleg(front_left,   0,    0,  20, 0);
          paul.settoleg(middle_left,  0,    0,    0, 0);
          paul.settoleg(rear_left,    0,    0,  20, 0);
          paul.settoleg(front_right,  0,    0,    0, 0);
          paul.settoleg(middle_right, 0,    0,  20, 0);
          paul.settoleg(rear_right,   0,    0,    0, 0);
          paul.movelegs(speed);

          paul.settoleg(front_left,   0,    0,  -20, 0);
          paul.settoleg(middle_left,  0,    0,    0, 0);
          paul.settoleg(rear_left,    0,    0,  -20, 0);
          paul.settoleg(front_right,  0,    0,    0, 0);
          paul.settoleg(middle_right, 0,    0,  -20, 0);
          paul.settoleg(rear_right,   0,    0,    0, 0);
          paul.movelegs(speed);

          paul.settoleg(front_left,   0,    0,  20, 0);
          paul.settoleg(middle_left,  0,    0,    0, 0);
          paul.settoleg(rear_left,    0,    0,  20, 0);
          paul.settoleg(front_right,  0,    0,    0, 0);
          paul.settoleg(middle_right, 0,    0,  20, 0);
          paul.settoleg(rear_right,   0,    0,    0, 0);
          paul.movelegs(speed);

          //
          //      Linke Seite 4x Rauf runter
          //
          paul.settoleg(front_left,   0,    0,    0, 0);
          paul.settoleg(middle_left,  0,    0,  -20, 0);
          paul.settoleg(rear_left,    0,    0,    0, 0);
          paul.settoleg(front_right,  0,    0,  -20, 0);
          paul.settoleg(middle_right, 0,    0,    0, 0);
          paul.settoleg(rear_right,   0,    0,  -20, 0);
          paul.movelegs(speed);

          paul.settoleg(front_left,   0,    0,    0, 0);
          paul.settoleg(middle_left,  0,    0,   20, 0);
          paul.settoleg(rear_left,    0,    0,    0, 0);
          paul.settoleg(front_right,  0,    0,   20, 0);
          paul.settoleg(middle_right, 0,    0,    0, 0);
          paul.settoleg(rear_right,   0,    0,   20, 0);
          paul.movelegs(speed);


          paul.settoleg(front_left,   0,    0,    0, 0);
          paul.settoleg(middle_left,  0,    0,  -20, 0);
          paul.settoleg(rear_left,    0,    0,    0, 0);
          paul.settoleg(front_right,  0,    0,  -20, 0);
          paul.settoleg(middle_right, 0,    0,    0, 0);
          paul.settoleg(rear_right,   0,    0,  -20, 0);
          paul.movelegs(speed);

          paul.settoleg(front_left,   0,    0,    0, 0);
          paul.settoleg(middle_left,  0,    0,   20, 0);
          paul.settoleg(rear_left,    0,    0,    0, 0);
          paul.settoleg(front_right,  0,    0,   20, 0);
          paul.settoleg(middle_right, 0,    0,    0, 0);
          paul.settoleg(rear_right,   0,    0,   20, 0);
          paul.movelegs(speed);


          paul.settoleg(front_left,   0,    0,    0, 0);
          paul.settoleg(middle_left,  0,    0,  -20, 0);
          paul.settoleg(rear_left,    0,    0,    0, 0);
          paul.settoleg(front_right,  0,    0,  -20, 0);
          paul.settoleg(middle_right, 0,    0,    0, 0);
          paul.settoleg(rear_right,   0,    0,  -20, 0);
          paul.movelegs(speed);

          paul.settoleg(front_left,   0,    0,    0, 0);
          paul.settoleg(middle_left,  0,    0,   20, 0);
          paul.settoleg(rear_left,    0,    0,    0, 0);
          paul.settoleg(front_right,  0,    0,   20, 0);
          paul.settoleg(middle_right, 0,    0,    0, 0);
          paul.settoleg(rear_right,   0,    0,   20, 0);
          paul.movelegs(speed);

          paul.settoleg(front_left,   0,    0,    0, 0);
          paul.settoleg(middle_left,  0,    0,  -20, 0);
          paul.settoleg(rear_left,    0,    0,    0, 0);
          paul.settoleg(front_right,  0,    0,  -20, 0);
          paul.settoleg(middle_right, 0,    0,    0, 0);
          paul.settoleg(rear_right,   0,    0,  -20, 0);
          paul.movelegs(speed);

          paul.settoleg(front_left,   0,    0,    0, 0);
          paul.settoleg(middle_left,  0,    0,   20, 0);
          paul.settoleg(rear_left,    0,    0,    0, 0);
          paul.settoleg(front_right,  0,    0,   20, 0);
          paul.settoleg(middle_right, 0,    0,    0, 0);
          paul.settoleg(rear_right,   0,    0,   20, 0);
          paul.movelegs(speed);

          //
          //  Rechts Links Recht Links
          //

          paul.settoleg(front_left,   0,    0,  -20, 0);
          paul.settoleg(middle_left,  0,    0,    0, 0);
          paul.settoleg(rear_left,    0,    0,  -20, 0);
          paul.settoleg(front_right,  0,    0,    0, 0);
          paul.settoleg(middle_right, 0,    0,  -20, 0);
          paul.settoleg(rear_right,   0,    0,    0, 0);
          paul.movelegs(speed);

          paul.settoleg(front_left,   0,    0,  20, 0);
          paul.settoleg(middle_left,  0,    0,    0, 0);
          paul.settoleg(rear_left,    0,    0,  20, 0);
          paul.settoleg(front_right,  0,    0,    0, 0);
          paul.settoleg(middle_right, 0,    0,  20, 0);
          paul.settoleg(rear_right,   0,    0,    0, 0);
          paul.movelegs(speed);

          paul.settoleg(front_left,   0,    0,    0, 0);
          paul.settoleg(middle_left,  0,    0,  -20, 0);
          paul.settoleg(rear_left,    0,    0,    0, 0);
          paul.settoleg(front_right,  0,    0,  -20, 0);
          paul.settoleg(middle_right, 0,    0,    0, 0);
          paul.settoleg(rear_right,   0,    0,  -20, 0);
          paul.movelegs(speed);

          paul.settoleg(front_left,   0,    0,    0, 0);
          paul.settoleg(middle_left,  0,    0,   20, 0);
          paul.settoleg(rear_left,    0,    0,    0, 0);
          paul.settoleg(front_right,  0,    0,   20, 0);
          paul.settoleg(middle_right, 0,    0,    0, 0);
          paul.settoleg(rear_right,   0,    0,   20, 0);
          paul.movelegs(speed);

          paul.settoleg(front_left,   0,    0,  -20, 0);
          paul.settoleg(middle_left,  0,    0,    0, 0);
          paul.settoleg(rear_left,    0,    0,  -20, 0);
          paul.settoleg(front_right,  0,    0,    0, 0);
          paul.settoleg(middle_right, 0,    0,  -20, 0);
          paul.settoleg(rear_right,   0,    0,    0, 0);
          paul.movelegs(speed);

          paul.settoleg(front_left,   0,    0,  20, 0);
          paul.settoleg(middle_left,  0,    0,    0, 0);
          paul.settoleg(rear_left,    0,    0,  20, 0);
          paul.settoleg(front_right,  0,    0,    0, 0);
          paul.settoleg(middle_right, 0,    0,  20, 0);
          paul.settoleg(rear_right,   0,    0,    0, 0);
          paul.movelegs(speed);

          paul.settoleg(front_left,   0,    0,    0, 0);
          paul.settoleg(middle_left,  0,    0,  -20, 0);
          paul.settoleg(rear_left,    0,    0,    0, 0);
          paul.settoleg(front_right,  0,    0,  -20, 0);
          paul.settoleg(middle_right, 0,    0,    0, 0);
          paul.settoleg(rear_right,   0,    0,  -20, 0);
          paul.movelegs(speed);

          paul.settoleg(front_left,   0,    0,    0, 0);
          paul.settoleg(middle_left,  0,    0,   20, 0);
          paul.settoleg(rear_left,    0,    0,    0, 0);
          paul.settoleg(front_right,  0,    0,   20, 0);
          paul.settoleg(middle_right, 0,    0,    0, 0);
          paul.settoleg(rear_right,   0,    0,   20, 0);
          paul.movelegs(speed);


          paul.introwalk(10, 0, 20, -15);
          paul.walk(21, 10, 0, 20, -15);
          paul.outrowalk(10, 0, 20, -15);

        }
        funk.sendData(&yes);

        break;

      case 5:

        //
        //      Rechte Seite 4x Rauf runter
        //
        paul.settoleg(front_left,   0,    0,  -20, 0);
        paul.settoleg(middle_left,  0,    0,    0, 0);
        paul.settoleg(rear_left,    0,    0,  -20, 0);
        paul.settoleg(front_right,  0,    0,    0, 0);
        paul.settoleg(middle_right, 0,    0,  -20, 0);
        paul.settoleg(rear_right,   0,    0,    0, 0);
        paul.movelegs(speed);

        paul.settoleg(front_left,   0,    0,  20, 0);
        paul.settoleg(middle_left,  0,    0,    0, 0);
        paul.settoleg(rear_left,    0,    0,  20, 0);
        paul.settoleg(front_right,  0,    0,    0, 0);
        paul.settoleg(middle_right, 0,    0,  20, 0);
        paul.settoleg(rear_right,   0,    0,    0, 0);
        paul.movelegs(speed);

        paul.settoleg(front_left,   0,    0,  -20, 0);
        paul.settoleg(middle_left,  0,    0,    0, 0);
        paul.settoleg(rear_left,    0,    0,  -20, 0);
        paul.settoleg(front_right,  0,    0,    0, 0);
        paul.settoleg(middle_right, 0,    0,  -20, 0);
        paul.settoleg(rear_right,   0,    0,    0, 0);
        paul.movelegs(speed);

        paul.settoleg(front_left,   0,    0,  20, 0);
        paul.settoleg(middle_left,  0,    0,    0, 0);
        paul.settoleg(rear_left,    0,    0,  20, 0);
        paul.settoleg(front_right,  0,    0,    0, 0);
        paul.settoleg(middle_right, 0,    0,  20, 0);
        paul.settoleg(rear_right,   0,    0,    0, 0);
        paul.movelegs(speed);

        paul.settoleg(front_left,   0,    0,  -20, 0);
        paul.settoleg(middle_left,  0,    0,    0, 0);
        paul.settoleg(rear_left,    0,    0,  -20, 0);
        paul.settoleg(front_right,  0,    0,    0, 0);
        paul.settoleg(middle_right, 0,    0,  -20, 0);
        paul.settoleg(rear_right,   0,    0,    0, 0);
        paul.movelegs(speed);

        paul.settoleg(front_left,   0,    0,  20, 0);
        paul.settoleg(middle_left,  0,    0,    0, 0);
        paul.settoleg(rear_left,    0,    0,  20, 0);
        paul.settoleg(front_right,  0,    0,    0, 0);
        paul.settoleg(middle_right, 0,    0,  20, 0);
        paul.settoleg(rear_right,   0,    0,    0, 0);
        paul.movelegs(speed);

        paul.settoleg(front_left,   0,    0,  -20, 0);
        paul.settoleg(middle_left,  0,    0,    0, 0);
        paul.settoleg(rear_left,    0,    0,  -20, 0);
        paul.settoleg(front_right,  0,    0,    0, 0);
        paul.settoleg(middle_right, 0,    0,  -20, 0);
        paul.settoleg(rear_right,   0,    0,    0, 0);
        paul.movelegs(speed);

        paul.settoleg(front_left,   0,    0,  20, 0);
        paul.settoleg(middle_left,  0,    0,    0, 0);
        paul.settoleg(rear_left,    0,    0,  20, 0);
        paul.settoleg(front_right,  0,    0,    0, 0);
        paul.settoleg(middle_right, 0,    0,  20, 0);
        paul.settoleg(rear_right,   0,    0,    0, 0);
        paul.movelegs(speed);

        //
        //      Linke Seite 4x Rauf runter
        //
        paul.settoleg(front_left,   0,    0,    0, 0);
        paul.settoleg(middle_left,  0,    0,  -20, 0);
        paul.settoleg(rear_left,    0,    0,    0, 0);
        paul.settoleg(front_right,  0,    0,  -20, 0);
        paul.settoleg(middle_right, 0,    0,    0, 0);
        paul.settoleg(rear_right,   0,    0,  -20, 0);
        paul.movelegs(speed);

        paul.settoleg(front_left,   0,    0,    0, 0);
        paul.settoleg(middle_left,  0,    0,   20, 0);
        paul.settoleg(rear_left,    0,    0,    0, 0);
        paul.settoleg(front_right,  0,    0,   20, 0);
        paul.settoleg(middle_right, 0,    0,    0, 0);
        paul.settoleg(rear_right,   0,    0,   20, 0);
        paul.movelegs(speed);


        paul.settoleg(front_left,   0,    0,    0, 0);
        paul.settoleg(middle_left,  0,    0,  -20, 0);
        paul.settoleg(rear_left,    0,    0,    0, 0);
        paul.settoleg(front_right,  0,    0,  -20, 0);
        paul.settoleg(middle_right, 0,    0,    0, 0);
        paul.settoleg(rear_right,   0,    0,  -20, 0);
        paul.movelegs(speed);

        paul.settoleg(front_left,   0,    0,    0, 0);
        paul.settoleg(middle_left,  0,    0,   20, 0);
        paul.settoleg(rear_left,    0,    0,    0, 0);
        paul.settoleg(front_right,  0,    0,   20, 0);
        paul.settoleg(middle_right, 0,    0,    0, 0);
        paul.settoleg(rear_right,   0,    0,   20, 0);
        paul.movelegs(speed);


        paul.settoleg(front_left,   0,    0,    0, 0);
        paul.settoleg(middle_left,  0,    0,  -20, 0);
        paul.settoleg(rear_left,    0,    0,    0, 0);
        paul.settoleg(front_right,  0,    0,  -20, 0);
        paul.settoleg(middle_right, 0,    0,    0, 0);
        paul.settoleg(rear_right,   0,    0,  -20, 0);
        paul.movelegs(speed);

        paul.settoleg(front_left,   0,    0,    0, 0);
        paul.settoleg(middle_left,  0,    0,   20, 0);
        paul.settoleg(rear_left,    0,    0,    0, 0);
        paul.settoleg(front_right,  0,    0,   20, 0);
        paul.settoleg(middle_right, 0,    0,    0, 0);
        paul.settoleg(rear_right,   0,    0,   20, 0);
        paul.movelegs(speed);

        paul.settoleg(front_left,   0,    0,    0, 0);
        paul.settoleg(middle_left,  0,    0,  -20, 0);
        paul.settoleg(rear_left,    0,    0,    0, 0);
        paul.settoleg(front_right,  0,    0,  -20, 0);
        paul.settoleg(middle_right, 0,    0,    0, 0);
        paul.settoleg(rear_right,   0,    0,  -20, 0);
        paul.movelegs(speed);

        paul.settoleg(front_left,   0,    0,    0, 0);
        paul.settoleg(middle_left,  0,    0,   20, 0);
        paul.settoleg(rear_left,    0,    0,    0, 0);
        paul.settoleg(front_right,  0,    0,   20, 0);
        paul.settoleg(middle_right, 0,    0,    0, 0);
        paul.settoleg(rear_right,   0,    0,   20, 0);
        paul.movelegs(speed);

        //
        //  Rechts Links Recht Links
        //

        paul.settoleg(front_left,   0,    0,  -20, 0);
        paul.settoleg(middle_left,  0,    0,    0, 0);
        paul.settoleg(rear_left,    0,    0,  -20, 0);
        paul.settoleg(front_right,  0,    0,    0, 0);
        paul.settoleg(middle_right, 0,    0,  -20, 0);
        paul.settoleg(rear_right,   0,    0,    0, 0);
        paul.movelegs(speed);

        paul.settoleg(front_left,   0,    0,  20, 0);
        paul.settoleg(middle_left,  0,    0,    0, 0);
        paul.settoleg(rear_left,    0,    0,  20, 0);
        paul.settoleg(front_right,  0,    0,    0, 0);
        paul.settoleg(middle_right, 0,    0,  20, 0);
        paul.settoleg(rear_right,   0,    0,    0, 0);
        paul.movelegs(speed);

        paul.settoleg(front_left,   0,    0,    0, 0);
        paul.settoleg(middle_left,  0,    0,  -20, 0);
        paul.settoleg(rear_left,    0,    0,    0, 0);
        paul.settoleg(front_right,  0,    0,  -20, 0);
        paul.settoleg(middle_right, 0,    0,    0, 0);
        paul.settoleg(rear_right,   0,    0,  -20, 0);
        paul.movelegs(speed);

        paul.settoleg(front_left,   0,    0,    0, 0);
        paul.settoleg(middle_left,  0,    0,   20, 0);
        paul.settoleg(rear_left,    0,    0,    0, 0);
        paul.settoleg(front_right,  0,    0,   20, 0);
        paul.settoleg(middle_right, 0,    0,    0, 0);
        paul.settoleg(rear_right,   0,    0,   20, 0);
        paul.movelegs(speed);

        paul.settoleg(front_left,   0,    0,  -20, 0);
        paul.settoleg(middle_left,  0,    0,    0, 0);
        paul.settoleg(rear_left,    0,    0,  -20, 0);
        paul.settoleg(front_right,  0,    0,    0, 0);
        paul.settoleg(middle_right, 0,    0,  -20, 0);
        paul.settoleg(rear_right,   0,    0,    0, 0);
        paul.movelegs(speed);

        paul.settoleg(front_left,   0,    0,  20, 0);
        paul.settoleg(middle_left,  0,    0,    0, 0);
        paul.settoleg(rear_left,    0,    0,  20, 0);
        paul.settoleg(front_right,  0,    0,    0, 0);
        paul.settoleg(middle_right, 0,    0,  20, 0);
        paul.settoleg(rear_right,   0,    0,    0, 0);
        paul.movelegs(speed);

        paul.settoleg(front_left,   0,    0,    0, 0);
        paul.settoleg(middle_left,  0,    0,  -20, 0);
        paul.settoleg(rear_left,    0,    0,    0, 0);
        paul.settoleg(front_right,  0,    0,  -20, 0);
        paul.settoleg(middle_right, 0,    0,    0, 0);
        paul.settoleg(rear_right,   0,    0,  -20, 0);
        paul.movelegs(speed);

        paul.settoleg(front_left,   0,    0,    0, 0);
        paul.settoleg(middle_left,  0,    0,   20, 0);
        paul.settoleg(rear_left,    0,    0,    0, 0);
        paul.settoleg(front_right,  0,    0,   20, 0);
        paul.settoleg(middle_right, 0,    0,    0, 0);
        paul.settoleg(rear_right,   0,    0,   20, 0);
        paul.movelegs(speed);


        paul.introwalk(10, 0, 20, -15);
        paul.walk(21, 10, 0, 20, -15);
        paul.outrowalk(10, 0, 20, -15);

        funk.sendData(&yes);
        break;

      case 6:

        paul.leg[middle_left].femurw = 160;
        paul.leg[middle_left].tibiaw = 160;

        paul.leg[middle_right].coaxw = -90;
        paul.leg[middle_right].femurw = 160;
        paul.leg[middle_right].tibiaw = 160;
        paul.writeServo(middle_left);
        paul.writeServo(middle_right);
        delay(300);


        paul.leg[middle_left].tibiaw = 140;

        paul.leg[middle_right].coaxw = -90;
        paul.leg[middle_right].femurw = 160;
        paul.leg[middle_right].tibiaw = 140;
        paul.writeServo(middle_left);
        paul.writeServo(middle_right);
        delay(300);

        paul.leg[middle_left].tibiaw = 160;
        paul.leg[middle_right].coaxw = -90;
        paul.leg[middle_right].femurw = 160;
        paul.leg[middle_right].tibiaw = 160;
        paul.writeServo(middle_left);
        paul.writeServo(middle_right);
        delay(300);

        paul.leg[middle_left].tibiaw = 140;
        paul.leg[middle_right].coaxw = -90;
        paul.leg[middle_right].femurw = 160;
        paul.leg[middle_right].tibiaw = 140;
        paul.writeServo(middle_left);
        paul.writeServo(middle_right);
        delay(300);

        paul.leg[middle_left].tibiaw = 160;
        paul.leg[middle_right].coaxw = -90;
        paul.leg[middle_right].femurw = 160;
        paul.leg[middle_right].tibiaw = 160;
        paul.writeServo(middle_left);
        paul.writeServo(middle_right);
        delay(300);

        paul.leg[middle_left].tibiaw = 140;
        paul.leg[middle_right].coaxw = -90;
        paul.leg[middle_right].femurw = 160;
        paul.leg[middle_right].tibiaw = 140;
        paul.writeServo(middle_left);
        paul.writeServo(middle_right);
        delay(300);

        paul.leg[middle_left].tibiaw = 160;
        paul.leg[middle_right].coaxw = -90;
        paul.leg[middle_right].femurw = 160;
        paul.leg[middle_right].tibiaw = 160;
        paul.writeServo(middle_left);
        paul.writeServo(middle_right);
        delay(300);

        paul.leg[middle_left].tibiaw = 140;
        paul.leg[middle_right].coaxw = -90;
        paul.leg[middle_right].femurw = 160;
        paul.leg[middle_right].tibiaw = 140;
        paul.writeServo(middle_left);
        paul.writeServo(middle_right);
        delay(300);

        paul.leg[middle_left].tibiaw = 160;
        paul.leg[middle_right].coaxw = -90;
        paul.leg[middle_right].femurw = 160;
        paul.leg[middle_right].tibiaw = 160;
        paul.writeServo(middle_left);
        paul.writeServo(middle_right);
        delay(300);



        paul.init_koor(front_left,    80, -65, 100, 57.362, -84.853, 0);
        paul.init_koor(middle_left,   100, 0, 100, 71.90, 0, 0);
        paul.init_koor(rear_left,     80, 65, 100, 57.362, 84.853, 0);
        paul.init_koor(front_right,   -80, -65, 100, -57.362, -84.853, 0);
        paul.init_koor(middle_right,  -100, 0, 100, -71.90, 0, 0);
        paul.init_koor(rear_right,    -80, 65, 100, -57.362, 84.853, 0);
        for (int legNumber = 0; legNumber < 6; legNumber++) {
          paul.calculateW((int)legNumber);
          paul.writeServo(legNumber);

        }
        delay(500);

        funk.sendData(&yes);
        break;

      case 7:

        break;

      case 8:
        speed = value[1];
        funk.sendData(&yes);
        break;

      case 9:
        paul.set_steps(value[1]);
        funk.sendData(&yes);
        break;

      default:
        funk.sendData(&noCommand);
        break;
    }
  }
}
