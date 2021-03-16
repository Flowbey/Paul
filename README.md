# Paul

Paul is a hexapod based on esp32 (wemos lolin D32 pro)

Features:
- Inverse kinematics
- Accelerate and brake included in the moves for realistic movment
- Radio controlled 2.4 GHz modul 
- Power 24A max peak for the servos

Controlling:
Send a string like "3,0,20,0,10,0" means command 3 for walk, 0 times repeat, 20mm forward, 0mm sideway, 10mm step high, 0 degre tourn
You can easly add more commands with the functions:

settoleg(uint8_t legNumber, int aplitudeX, int aplitudeY, int amplitudeZ, float yaw);     
movelegs(int _resolution);  

Comming soon:
Autonomous walk with a VL53L0 Laser distance sensor
