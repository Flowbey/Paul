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
    void attachServo( uint8_t servoNumber, uint8_t servoPin, int woffs, bool flipped);
    void init_leg(float coax, float femur, float tibia); // the lenght of coax, femur and tibia
    void init_leg_range(int _mincoaxw, int _maxcoaxw, int _minfemurw, int _maxfemurw, int _mintibiaw, int _maxtibiaw);
    void init_koor(uint8_t legNumber, float x, float y, float z, float xoffs, float yoffs, float zoffs); //start coordinates and offsets
    void set_steps(int steps);
    void set_new_koor(uint8_t legNumber, float x, float y, float z, float yaw); //go to this coordinates
    void move();


    void calculateW(uint8_t legNumber);
    int writeServo(uint8_t legNumber);
    int pulseWidth(int angle);
    int walk(int repeat, int aplitudeX, int aplitudeY, int amplitudeZ, float yaw);
    int introwalk(int aplitudeX, int aplitudeY, int amplitudeZ, float yaw);
    int outrowalk(int aplitudeX, int aplitudeY, int amplitudeZ, float yaw);
    int movetoall(int aplitudeX, int aplitudeY, int amplitudeZ, float yaw);
    int settoleg(uint8_t legNumber, int aplitudeX, int aplitudeY, int amplitudeZ, float yaw);
    int movelegs(int _resolution);



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
