#ifndef hexapod_h
#define hexapod_h

#include <Adafruit_PWMServoDriver.h>
#define MIN_PULSE_WIDTH 650
#define MAX_PULSE_WIDTH 2050
#define DEFAULT_PULSE_WIDTH 1500
#define FREQUENCY 50


class hexapod {
  public:
    hexapod() ;
    void begin();
    void attachServo( uint8_t servoNumber, uint8_t servoPin, int woffs, bool flipped);                                    // attach servos => number, servo pin, angle offset vor calibrating, flipped turned 180 degrees
    void init_leg(float coax, float femur, float tibia);                                                                  // set the lenght of coax, femur and tibia in mm
    void init_leg_range(int _mincoaxw, int _maxcoaxw, int _minfemurw, int _maxfemurw, int _mintibiaw, int _maxtibiaw);    // set max and min angle range for each joint 0 - 180
    void init_koor(uint8_t legNumber, float x, float y, float z, float xoffs, float yoffs, float zoffs);                  // start coordinates and offsets (shift from the center of body)
    void set_steps(int steps);                                                                                            // how many steps for calculating the moves (resolution)
    void set_new_koor(uint8_t legNumber, float x, float y, float z, float yaw);                                           // set new coordinates relative to current position for one leg no accleration!!
    void move();                                                                                                          // move all legs to te new coordinates no accleration!!
    int walk(int repeat, int aplitudeX, int aplitudeY, int amplitudeZ, float yaw);                                        // walk ;-)
    int introwalk(int aplitudeX, int aplitudeY, int amplitudeZ, float yaw);                                               // start walking with one step begining from homepoint
    int outrowalk(int aplitudeX, int aplitudeY, int amplitudeZ, float yaw);                                               // stop walking with one step ending to homepoint
    int movetoall(int aplitudeX, int aplitudeY, int amplitudeZ, float yaw);                                               // move all legs relative to to homepoint
    int settoleg(uint8_t legNumber, int aplitudeX, int aplitudeY, int amplitudeZ, float yaw);                             // set new coordinates relative to current position for one leg with accleration
    int movelegs(int _resolution);                                                                                        // move all legs to te new coordinates with accleration
    void calculateW(uint8_t legNumber);                                                                                   // calculate angel for servo with inverse kinematic
    int writeServo(uint8_t legNumber);                                                                                    // write angle to servo
    int pulseWidth(int angle);                                                                                            // calculate the pwm signal



    Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x40);
    Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x41);

    struct legs {
      float x;
      float y;
      float z;
      float xCurrent;
      float yCurrent;
      float zCurrent;
      float xHome;
      float yHome;
      float zHome;
      float coaxw;
      float femurw;
      float tibiaw;
      float yaw;
      float pitch;
      float bend;
      float yawCurrent;
      float pitchCurrent;
      float bendCurrent;
      float xoffs;
      float yoffs;
      float zoffs;
      float xstep;
      float ystep;
      float zstep;
      float yawstep;
      float pitchstep;
      float bendstep;
    };
    struct legs leg[6];

    struct servos {
      uint8_t pin;
      int woffs;
      bool flipped;
    };

    struct servos servo[18];
    float coax, femur, tibia ;
    float winkel[3];
    uint8_t assign_pin[6][3];
    int resolution;
    int tripod_case[6] = {1, 2, 1, 2, 1, 2};

  private:

    const float Pi = 3.14159265358979323846;

    int mincoaxw, maxcoaxw, minfemurw, maxfemurw, mintibiaw, maxtibiaw;



};



#endif
