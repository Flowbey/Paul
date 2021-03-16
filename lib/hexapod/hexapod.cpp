#include <Arduino.h>
#include "hexapod.h"
#include <math.h>


//#define DEBUGHEXA_ANGLE


//#define DEBUGHEXA_YAW
//#define DEBUGHEXA_STEP
//#define DEBUGHEXA3

hexapod::hexapod() {

}

void hexapod::begin() {
  pwm1.begin();
  pwm1.setPWMFreq(FREQUENCY);
  pwm2.begin();
  pwm2.setPWMFreq(FREQUENCY);

}

//======================================
//====================================== Attach servo
//======================================

void hexapod::attachServo( uint8_t servoNumber, uint8_t servoPin, int woffs, bool flipped) {
  servo[servoNumber].pin = servoPin;
  servo[servoNumber].woffs = woffs;
  servo[servoNumber].flipped = flipped;
}

//======================================
//====================================== Init leg (set the lenght of coax, femur and tibia in mm)
//======================================

void hexapod::init_leg(float _coax, float _femur, float _tibia ) {
  coax = _coax;
  femur = _femur;
  tibia = _tibia;
}

//======================================
//====================================== init leg range (set the min and max degrees for each joint)
//======================================

void hexapod::init_leg_range(int _mincoaxw, int _maxcoaxw, int _minfemurw, int _maxfemurw, int _mintibiaw, int _maxtibiaw) {
  mincoaxw = _mincoaxw;
  maxcoaxw = _maxcoaxw;
  minfemurw = _minfemurw;
  maxfemurw = _maxfemurw;
  mintibiaw = _mintibiaw;
  maxtibiaw = _maxtibiaw;
}

//======================================
//====================================== Init koordinate (set the start coordinates and set offsets, shift from the center of body)
//======================================

void hexapod::init_koor(uint8_t legNumber, float x, float y, float z, float xoffs, float yoffs, float zoffs) {
  leg[legNumber].x = x;
  leg[legNumber].y = y;
  leg[legNumber].z = z;
  leg[legNumber].yaw = 0;
  leg[legNumber].pitch = 0;
  leg[legNumber].bend = 0;
  leg[legNumber].xHome = x;
  leg[legNumber].yHome = y;
  leg[legNumber].zHome = z;
  leg[legNumber].xCurrent = x;
  leg[legNumber].yCurrent = y;
  leg[legNumber].zCurrent = z;
  leg[legNumber].yawCurrent = 0;
  leg[legNumber].pitchCurrent = 0;
  leg[legNumber].bendCurrent = 0;
  leg[legNumber].xstep = 0;
  leg[legNumber].ystep = 0;
  leg[legNumber].zstep = 0;
  leg[legNumber].yawstep = 0;
  leg[legNumber].pitchstep = 0;
  leg[legNumber].bendstep = 0;

  leg[legNumber].xoffs = xoffs;
  leg[legNumber].yoffs = yoffs;
  leg[legNumber].zoffs = zoffs;

#ifdef DEBUGHEXA_ANGLE
  Serial.print("Offsets Bein:");
  Serial.println(legNumber);
  Serial.print(" x:");
  Serial.print(leg[legNumber].xoffs);
  Serial.print(" y:");
  Serial.print(leg[legNumber].yoffs);
  Serial.print(" z:");
  Serial.println(leg[legNumber].zoffs);
#endif
}

//======================================
//====================================== set steps (resolution for calculating how many steps)
//======================================

void hexapod::set_steps(int steps) {
  resolution = steps;
}

//======================================
//====================================== Set new koordinate
//======================================

void hexapod::set_new_koor(uint8_t legNumber, float x, float y, float z, float yaw) {
  leg[legNumber].xstep = (x - leg[legNumber].x ) / resolution;
  leg[legNumber].ystep = (y - leg[legNumber].y ) / resolution;
  leg[legNumber].zstep = (z - leg[legNumber].z ) / resolution;
  leg[legNumber].yawstep = (yaw - leg[legNumber].yaw ) / resolution;
}
//======================================
//====================================== Move
//======================================

void hexapod::move() {
  //Steps Loop Auflösung in N Schritte

  for (uint8_t i = 0; i < resolution; i++) {

#ifdef DEBUGHEXA_STEP
    Serial.print("Step Nummer: ");
    Serial.println(i);
#endif

    //Leg Schritweise Koordinaten erhöhen für jedes Bein, danach an PWM shield senden
    for (uint8_t ii = 0; ii < 6; ii++) {
      leg[ii].x = leg[ii].x + leg[ii].xstep ;
      leg[ii].y = leg[ii].y + leg[ii].ystep ;
      leg[ii].z = leg[ii].z + leg[ii].zstep ;
      leg[ii].yaw = leg[ii].yaw + leg[ii].yawstep ;
      calculateW(ii);
      writeServo(ii);
    }
  }
}

//======================================
//====================================== calculate angle
//======================================

void hexapod::calculateW(uint8_t legNumber) {

  // Bodi IK yaw mit einberechnen. Drehversatz der x und y Koordinaten vom Körpermittelpunkt aus. Körpermittelpunkt wird mit den Offsets zurück gerechnet.
  // Yaw Winkel für die Linke seite invertieren

  float _x = ((leg[legNumber].x + leg[legNumber].xoffs) * cos(leg[legNumber].yaw / 180 * Pi) - (leg[legNumber].y + leg[legNumber].yoffs) * sin(leg[legNumber].yaw / 180 * Pi) - leg[legNumber].xoffs) ;
  float _y = ((leg[legNumber].x + leg[legNumber].xoffs) * sin(leg[legNumber].yaw / 180 * Pi) + (leg[legNumber].y + leg[legNumber].yoffs) * cos(leg[legNumber].yaw / 180 * Pi) - leg[legNumber].yoffs) ;
  float _z = leg[legNumber].z;

#ifdef DEBUGHEXA_YAW
  Serial.print("Bein mit Winkel: ");
  Serial.print(legNumber);
  Serial.print(" .  x . ");
  Serial.print(_x);
  Serial.print(" .  y . ");
  Serial.print(_y);
  Serial.print(" .  z . ");
  Serial.println(_z);
#endif

  float L1;
  float L;

  L1 = sqrt(sq(_x) + sq(_y));

  //if it not in Range abort
  if (L1 >= (coax + femur + tibia)) {
    Serial.println("Koordinate nicht im Rahmen");
    return ;
  }

  L = sqrt(sq(_z) + sq(L1 - coax));

  //Gamma = w_hip
  leg[legNumber].coaxw = atan2(_x, _y) * 180 / Pi;

  //Alpha = w_knee
  leg[legNumber].femurw = acos(_z / L) * 180 / Pi + acos((sq(tibia) - sq(femur) - sq(L)) / (-2 * femur * L)) * 180 / Pi;

  //Beta = w_foot
  leg[legNumber].tibiaw  = acos((sq(L) - sq(tibia) - sq(femur)) / (-2 * tibia * femur)) * 180 / Pi;
}

//======================================
//====================================== write to Servo
//======================================

int hexapod::writeServo(uint8_t legNumber) {
  uint8_t coaxServoNumber   =  (legNumber * 3) + 0;
  uint8_t femurServoNumber  = (legNumber * 3) + 1;
  uint8_t tibiaServoNumber  = (legNumber * 3) + 2;

  //Check if Servo flipped
  if (servo[coaxServoNumber].flipped) leg[legNumber].coaxw = leg[legNumber].coaxw + 180;
  if (servo[femurServoNumber].flipped) leg[legNumber].femurw = 180 - leg[legNumber].femurw;
  if (servo[tibiaServoNumber].flipped) leg[legNumber].tibiaw = 180 - leg[legNumber].tibiaw;

#ifdef DEBUGHEXA_ANGLE
  Serial.print("Raw   Nummer: ");
  Serial.print(legNumber);
  Serial.print(" .  x . ");
  Serial.print(leg[legNumber].x);
  Serial.print(" .  y . ");
  Serial.print(leg[legNumber].y);
  Serial.print(" .  z . ");
  Serial.print(leg[legNumber].z);
  Serial.print(" .  hip . ");
  Serial.print(leg[legNumber].coaxw);
  Serial.print(" .  knee . ");
  Serial.print(leg[legNumber].femurw);
  Serial.print(" .  foot . ");
  Serial.println(leg[legNumber].tibiaw);
#endif

  // add offset +/- depends if flipped or not
  leg[legNumber].coaxw += (servo[coaxServoNumber].flipped ? -servo[coaxServoNumber].woffs : servo[coaxServoNumber].woffs);
  leg[legNumber].femurw += (servo[femurServoNumber].flipped ? -servo[femurServoNumber].woffs : servo[femurServoNumber].woffs);
  leg[legNumber].tibiaw += (servo[tibiaServoNumber].flipped ? -servo[tibiaServoNumber].woffs : servo[tibiaServoNumber].woffs);

  if (mincoaxw >= leg[legNumber].coaxw || leg[legNumber].coaxw >= maxcoaxw) {
    Serial.print("Coax Winkel auserhalb Range");

    return 0;
  }
  if (minfemurw >= leg[legNumber].femurw || leg[legNumber].femurw >= maxfemurw) {
    Serial.print("Femur Winkel auserhalb Range");

    return 0;
  }
  if (mintibiaw >= leg[legNumber].tibiaw || leg[legNumber].tibiaw >= maxtibiaw) {
    Serial.print("Tibia Winkel auserhalb Range");

    return 0;
  }

  //decide wich pwm shield to send the signal depends on pin Number
  if (servo[coaxServoNumber].pin <= 15) {
    pwm1.setPWM(servo[coaxServoNumber].pin, 0 , pulseWidth(leg[legNumber].coaxw));
  }
  else {
    pwm2.setPWM(servo[coaxServoNumber].pin - 16, 0 , pulseWidth(leg[legNumber].coaxw));
  }

  if (servo[femurServoNumber].pin <= 15) {
    pwm1.setPWM(servo[femurServoNumber].pin, 0 , pulseWidth(leg[legNumber].femurw));
  }
  else {
    pwm2.setPWM(servo[femurServoNumber].pin - 16, 0 , pulseWidth(leg[legNumber].femurw));
  }

  if (servo[tibiaServoNumber].pin <= 15) {
    pwm1.setPWM(servo[tibiaServoNumber].pin, 0 , pulseWidth(leg[legNumber].tibiaw));
  }
  else {
    pwm2.setPWM(servo[tibiaServoNumber].pin - 16, 0 , pulseWidth(leg[legNumber].tibiaw));
  }

#ifdef DEBUGHEXA_ANGLE
  Serial.print("Beine Nummer: ");
  Serial.print(legNumber);
  Serial.print(" .  x . ");
  Serial.print(leg[legNumber].x);
  Serial.print(" .  y . ");
  Serial.print(leg[legNumber].y);
  Serial.print(" .  z . ");
  Serial.print(leg[legNumber].z);
  Serial.print(" .  hip . ");
  Serial.print(leg[legNumber].coaxw);
  Serial.print(" .  knee . ");
  Serial.print(leg[legNumber].femurw);
  Serial.print(" .  foot . ");
  Serial.println(leg[legNumber].tibiaw);
#endif

#ifdef DEBUGHEXA3
  Serial.print("Beine Nummer: ");
  Serial.print(legNumber);
  Serial.print(" .  x . ");
  Serial.print(leg[legNumber].x);
  Serial.print(" .  y . ");
  Serial.print(leg[legNumber].y);
  Serial.print(" .  z . ");
  Serial.println(leg[legNumber].z);

#endif
  return 1;
}

//======================================
// ===================================== Pulse Widht
//======================================

int hexapod::pulseWidth(int angle)
{
  int pulse_wide, analog_value;
  pulse_wide = map(angle, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  analog_value = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096);
  return analog_value;
}

//======================================
//====================================== WALK
//======================================

int hexapod::walk(int repeat , int amplitudeX, int amplitudeY, int amplitudeZ, float yaw ) {
  //int amplitudeX = 10;
  //int amplitudeY = 20;
  //int amplitudeZ = 20;

  for (int ii = 0 ; ii < (repeat * 2); ii++) {
    for (int i = 1 ; i < resolution + 1; i++) {

#ifdef DEBUGHEXA_STEP
      Serial.print("Step Nummer: ");
      Serial.println(i);
#endif

      for (int legNumber = 0 ; legNumber < 6 ; legNumber++) {
        switch (tripod_case[legNumber]) {

          // Legs on the ground
          case 1:
            leg[legNumber].y = leg[legNumber].yHome - amplitudeY * cos(M_PI * i / resolution);
            leg[legNumber].x = leg[legNumber].xHome + amplitudeX * cos(M_PI * i / resolution);
            leg[legNumber].z = leg[legNumber].zHome;
            leg[legNumber].yaw = leg[legNumber].yaw - (yaw  / resolution);
            if (i >= resolution) tripod_case[legNumber] = 2;
            break;

          // Legs to lift
          case 2:
            leg[legNumber].y = leg[legNumber].yHome + amplitudeY * cos(M_PI * i / resolution);
            leg[legNumber].x = leg[legNumber].xHome - amplitudeX * cos(M_PI * i / resolution);
            leg[legNumber].z = leg[legNumber].zHome - amplitudeZ * sin(M_PI * i / resolution);
            leg[legNumber].yaw = leg[legNumber].yaw + (yaw  / resolution);
            if (i >= resolution) tripod_case[legNumber] = 1;
            break;
        }
#ifdef DEBUGHEXA_YAW
        Serial.print("yaw: ");
        Serial.println(leg[legNumber].yaw);
#endif
        calculateW(legNumber);
        if (writeServo(legNumber) == 0) {
          return 0;
        }
      }
    }
  }
  return 1;
}

//======================================
//====================================== intro walk
//======================================

int hexapod::introwalk(int amplitudeX, int amplitudeY, int amplitudeZ, float yaw ) {
  //int amplitudeX = 10;
  //int amplitudeY = 20;
  //int amplitudeZ = 20;



  for (int i = 1 ; i < resolution + 1; i++) {

#ifdef DEBUGHEXA_STEP
    Serial.print("Step Nummer: ");
    Serial.println(i);
#endif


    for (int legNumber = 0 ; legNumber < 6 ; legNumber++) {
      switch (tripod_case[legNumber]) {

        // Legs to lift
        case 1:
          leg[legNumber].y = leg[legNumber].yHome - amplitudeY * sin((M_PI * i / resolution) / 2);
          leg[legNumber].x = leg[legNumber].xHome + amplitudeX * sin((M_PI * i / resolution) / 2);
          leg[legNumber].z = leg[legNumber].zHome - amplitudeZ * sin(M_PI * i / resolution);
          leg[legNumber].yaw = leg[legNumber].yaw + (yaw  / resolution);
          //if (i >= resolution) tripod_case[legNumber] = 2;
          break;

        // Legs on the ground
        case 2:
          leg[legNumber].y = leg[legNumber].yHome + amplitudeY * sin((M_PI * i / resolution) / 2);
          leg[legNumber].x = leg[legNumber].xHome - amplitudeX * sin((M_PI * i / resolution) / 2);
          leg[legNumber].yaw = leg[legNumber].yaw - (yaw  / resolution);
          //if (i >= resolution) tripod_case[legNumber] = 1;
          break;
      }
#ifdef DEBUGHEXA_YAW
      Serial.print("yaw: ");
      Serial.println(leg[legNumber].yaw);
#endif
      calculateW(legNumber);
      if (writeServo(legNumber) == 0) {
        return 0;
      }
    }
  }
  return 1;
}

//======================================
//====================================== Outro WALK
//======================================

int hexapod::outrowalk(int amplitudeX, int amplitudeY, int amplitudeZ, float yaw ) {
  //int amplitudeX = 10;
  //int amplitudeY = 20;
  //int amplitudeZ = 20;



  for (int i = 1 ; i < resolution + 1; i++) {

#ifdef DEBUGHEXA_STEP
    Serial.print("Step Nummer: ");
    Serial.println(i);
#endif


    for (int legNumber = 0 ; legNumber < 6 ; legNumber++) {
      switch (tripod_case[legNumber]) {

        // Legs on the ground
        case 1:
          leg[legNumber].y = leg[legNumber].yHome - amplitudeY * cos((M_PI * i / resolution) / 2);
          leg[legNumber].x = leg[legNumber].xHome + amplitudeX * cos((M_PI * i / resolution) / 2);
          leg[legNumber].yaw = leg[legNumber].yaw - (yaw  / resolution);
          //if (i >= resolution) tripod_case[legNumber] = 2;
          break;

        // Legs to lift
        case 2:
          leg[legNumber].y = leg[legNumber].yHome + amplitudeY * cos((M_PI * i / resolution) / 2);
          leg[legNumber].x = leg[legNumber].xHome - amplitudeX * cos((M_PI * i / resolution) / 2);
          leg[legNumber].z = leg[legNumber].zHome - amplitudeZ * sin(M_PI * i / resolution);
          leg[legNumber].yaw = leg[legNumber].yaw + (yaw  / resolution);
          //if (i >= resolution) tripod_case[legNumber] = 1;
          break;
      }
#ifdef DEBUGHEXA_YAW
      Serial.print("yaw: ");
      Serial.println(leg[legNumber].yaw);
#endif
      calculateW(legNumber);
      if (writeServo(legNumber) == 0) {
        return 0;
      }
    }
  }

  return 1;
}

//======================================
//====================================== move all legs relative to to homepoint
//======================================

int hexapod::movetoall(int amplitudeX, int amplitudeY, int amplitudeZ, float yaw ) {

  for (int legNumber = 0 ; legNumber < 6 ; legNumber++) {
    leg[legNumber].xCurrent = leg[legNumber].x;
    leg[legNumber].yCurrent = leg[legNumber].y;
    leg[legNumber].zCurrent = leg[legNumber].z;
    leg[legNumber].yawCurrent = leg[legNumber].yaw;
    leg[legNumber].pitchCurrent = leg[legNumber].pitch;
    leg[legNumber].bendCurrent = leg[legNumber].bend;

  }


  for (int i = 1 ; i < resolution + 1; i++) {

#ifdef DEBUGHEXA_STEP
    Serial.print("Step Nummer: ");
    Serial.println(i);
#endif


    for (int legNumber = 0 ; legNumber < 6 ; legNumber++) {

      leg[legNumber].x = leg[legNumber].xCurrent + (amplitudeX / 2) * (cos((M_PI * i / resolution) + M_PI) + 1) ;
      leg[legNumber].y = leg[legNumber].yCurrent + (amplitudeY / 2) * (cos((M_PI * i / resolution) + M_PI) + 1) ;
      leg[legNumber].z = leg[legNumber].zCurrent + (amplitudeZ / 2) * (cos((M_PI * i / resolution) + M_PI) + 1) ;
      leg[legNumber].yaw = leg[legNumber].yawCurrent - (yaw / 2) * (cos((M_PI * i / resolution) + M_PI) + 1) ;


#ifdef DEBUGHEXA_YAW
      Serial.print("yaw: ");
      Serial.println(leg[legNumber].yaw);
#endif
      calculateW(legNumber);
      if (writeServo(legNumber) == 0) {
        return 0;
      }
    }
  }

  return 1;
}

//======================================
//====================================== set new coordinates relative to current position
//======================================

int hexapod::settoleg(uint8_t legNumber, int amplitudeX, int amplitudeY, int amplitudeZ, float yaw ) {

  leg[legNumber].xCurrent = leg[legNumber].x;
  leg[legNumber].yCurrent = leg[legNumber].y;
  leg[legNumber].zCurrent = leg[legNumber].z;
  leg[legNumber].yawCurrent = leg[legNumber].yaw;
  leg[legNumber].pitchCurrent = leg[legNumber].pitch;
  leg[legNumber].bendCurrent = leg[legNumber].bend;

  leg[legNumber].xstep = amplitudeX;
  leg[legNumber].ystep = amplitudeY;
  leg[legNumber].zstep = amplitudeZ;
  leg[legNumber].yawstep = yaw;
  // leg[legNumber].pitchstep = pitch;
  // leg[legNumber].bendstep = bend;

  return 1;
}

//======================================
//====================================== Move legs
//======================================

int hexapod::movelegs(int _resolution) {


  for (int i = 1 ; i < _resolution + 1; i++) {

#ifdef DEBUGHEXA_STEP
    Serial.print("Step Nummer: ");
    Serial.println(i);
#endif


    for (int legNumber = 0 ; legNumber < 6 ; legNumber++) {

      leg[legNumber].x = leg[legNumber].xCurrent + (leg[legNumber].xstep)  * (cos((M_PI * i / resolution) + M_PI) + 1) ;
      leg[legNumber].y = leg[legNumber].yCurrent + (leg[legNumber].ystep)  * (cos((M_PI * i / resolution) + M_PI) + 1) ;
      leg[legNumber].z = leg[legNumber].zCurrent + (leg[legNumber].zstep)  * (cos((M_PI * i / resolution) + M_PI) + 1) ;
      leg[legNumber].yaw = leg[legNumber].yawCurrent - (leg[legNumber].yawstep)  * (cos((M_PI * i / resolution) + M_PI) + 1) ;
      // leg[legNumber].pitch = leg[legNumber].pitchCurrent - (pitch)  * (cos((M_PI * i / resolution) + M_PI) + 1) ;
      // leg[legNumber].bend = leg[legNumber].bendCurrent - (bend)  * (cos((M_PI * i / resolution) + M_PI) + 1) ;


#ifdef DEBUGHEXA_YAW
      Serial.print("yaw: ");
      Serial.println(leg[legNumber].yaw);
#endif
      calculateW(legNumber);
      if (writeServo(legNumber) == 0) {
        return 0;
      }
    }
  }

  for (int legNumber = 0 ; legNumber < 6 ; legNumber++) {
    leg[legNumber].xCurrent = leg[legNumber].x;
    leg[legNumber].yCurrent = leg[legNumber].y;
    leg[legNumber].zCurrent = leg[legNumber].z;
    leg[legNumber].yawCurrent = leg[legNumber].yaw;
    leg[legNumber].pitchCurrent = leg[legNumber].pitch;
    leg[legNumber].bendCurrent = leg[legNumber].bend;
  }


  return 1;
}
