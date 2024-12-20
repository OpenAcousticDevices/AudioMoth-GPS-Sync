# AudioMoth-GPS-Sync
Additional firmware for AudioMoth devices to generate WAV files synchronised with GPS time.

For more details, visit [AudioMoth GPS Sync](https://www.openacousticdevices.info/gps-sync).

Compatible with the [AudioMoth GPS Sync App](https://github.com/OpenAcousticDevices/AudioMoth-GPS-Sync-App).

### Usage ####

Clone the contents of [AudioMoth-Project](https://github.com/OpenAcousticDevices/AudioMoth-Project).

Replace the ```src/main.c``` from AudioMoth-Project with the ```src/main.c``` from this repository. Put all the remaining ```src/*.c``` files and all the ```src/*.h``` files from this repository into the ```/src/``` and ```/inc/``` folders of the AudioMoth-Project repository. Add the  ```/gps/``` folder from the AudioMoth-Project code into the compilation chain by updating the definitions of the include and source files in the ```/build/Makefile``` as below:

```
INC = ../cmsis ../device/inc ../emlib/inc ../emusb/inc ../drivers/inc ../fatfs/inc  ../gps/inc ../inc
SRC = ../device/src ../emlib/src ../emusb/src ../drivers/src ../fatfs/src  ../gps/src ../src
```

### License ###

Copyright 2023 [Open Acoustic Devices](http://www.openacousticdevices.info/).

[MIT license](http://www.openacousticdevices.info/license).
