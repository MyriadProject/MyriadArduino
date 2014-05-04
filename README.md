# Quickstart

1. Install Arduino IDE
2. Install picocom
3. Make virtualenv
4. Install requirements.txt into virtualenv
5. Move Robot_Control to Z_Robot_Control in Arduino core libraries (this is 
   because the ino build will find SPI.h in Robot_Control first instead of the one we need in the actual SPI library.)
6. Build and upload (ie, `ino clean && ino build && ino upload`)
